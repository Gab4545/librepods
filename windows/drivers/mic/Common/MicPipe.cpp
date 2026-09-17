/*++

Module Name:

    MicPipe.cpp

Abstract:

    Implementation of the LibrePods hi-res microphone bridge. See MicPipe.h.

    Design: a single global byte ring buffer guarded by a spin lock (the virtual
    mic is single-instance). User mode writes decoded PCM via the control device
    IOCTL; the ACX capture stream engine reads a packet per tick. No WPP tracing
    here (keeps the file self-contained).

Environment:

    Kernel mode

--*/

#include "MicPipe.h"

//
// ~170 ms of headroom @ 48 kHz mono 16-bit (96000 B/s) — enough to absorb
// Bluetooth packet jitter without becoming an appreciable delay on its own.
// Lives in the driver's non-paged image data, so it's safe to touch at
// DISPATCH_LEVEL.
//
// LATENCY FIX (was 0x20000u / ~1.36 s): the write side (BT hi-res uplink,
// decoded on its own schedule) and the read side (Windows' ACX capture
// timer, StreamEngine::ScheduleNextPass) run on two independent clocks with
// no drift correction between them. With only "drop oldest on hard overflow"
// as backpressure, the ring would creep up to full within a few seconds and
// then sit pinned there, turning what's meant to be a small jitter cushion
// into a fixed delay roughly equal to the buffer's entire capacity. Shrinking
// the ring bounds the worst case; MIC_RING_HIGH_WATERMARK/MIC_RING_TARGET_BYTES
// below add active trimming so steady-state latency stays near the target
// instead of drifting up to MIC_RING_BYTES.
//
#define MIC_RING_BYTES 0x4000u

//
// If buffered audio grows past this on a write, trim back down to
// MIC_RING_TARGET_BYTES instead of waiting for a hard overflow. This is what
// actually bounds steady-state latency — MIC_RING_BYTES only bounds the
// worst case before a hard drop.
//
#define MIC_RING_HIGH_WATERMARK 0x1800u  // ~64 ms
#define MIC_RING_TARGET_BYTES   0x0C00u  // ~32 ms

static UCHAR      g_Ring[MIC_RING_BYTES];
static ULONG      g_Head;   // next write index
static ULONG      g_Tail;   // next read index
static ULONG      g_Count;  // bytes currently buffered
static KSPIN_LOCK g_Lock;
static BOOLEAN    g_Inited = FALSE;

static WDFDEVICE  g_ControlDevice = NULL;

// Advances on every capture pull (MicPipeRead). The tray polls it to tell when
// an app is actually recording from the mic, and auto-enables/disables the
// hi-res stream accordingly.
static volatile LONG g_ReadTick = 0;

EXTERN_C_START

VOID
MicPipeInit(
    VOID
)
{
    if (g_Inited) {
        return;
    }
    KeInitializeSpinLock(&g_Lock);
    g_Head = g_Tail = g_Count = 0;
    g_Inited = TRUE;
}

VOID
MicPipeWrite(
    _In_reads_bytes_(Len) PVOID Data,
    _In_                  ULONG Len
)
{
    KIRQL  irql;
    PUCHAR src = (PUCHAR)Data;
    ULONG  i;

    if (!g_Inited || Data == NULL || Len == 0) {
        return;
    }

    KeAcquireSpinLock(&g_Lock, &irql);
    for (i = 0; i < Len; i++) {
        if (g_Count == MIC_RING_BYTES) {
            // Full: drop the oldest byte so the newest audio always wins.
            g_Tail = (g_Tail + 1u) % MIC_RING_BYTES;
            g_Count--;
        }
        g_Ring[g_Head] = src[i];
        g_Head = (g_Head + 1u) % MIC_RING_BYTES;
        g_Count++;
    }

    //
    // Active trim: if the write side is outpacing the read side (the two run
    // on independent clocks — see the MIC_RING_BYTES comment above), don't
    // let the backlog quietly grow. Once buffered audio exceeds the high
    // watermark, jump the tail forward to bring it back down to the target
    // level in one step. This is a deliberate, audible-if-you-time-it skip
    // rather than a slow creep, but it keeps steady-state latency bounded
    // near MIC_RING_TARGET_BYTES instead of drifting up toward
    // MIC_RING_BYTES.
    //
    if (g_Count > MIC_RING_HIGH_WATERMARK) {
        ULONG excess = g_Count - MIC_RING_TARGET_BYTES;
        g_Tail = (g_Tail + excess) % MIC_RING_BYTES;
        g_Count = MIC_RING_TARGET_BYTES;
    }

    KeReleaseSpinLock(&g_Lock, irql);
}

VOID
MicPipeRead(
    _Out_writes_bytes_(Len) PVOID Out,
    _In_                    ULONG Len
)
{
    KIRQL  irql;
    PUCHAR dst = (PUCHAR)Out;
    ULONG  i;

    if (Out == NULL || Len == 0) {
        return;
    }
    // A capture is pulling data: mark activity so the tray can auto-enable.
    InterlockedIncrement(&g_ReadTick);
    if (!g_Inited) {
        RtlZeroMemory(Out, Len);
        return;
    }

    KeAcquireSpinLock(&g_Lock, &irql);
    for (i = 0; i < Len; i++) {
        if (g_Count == 0) {
            dst[i] = 0;  // underrun -> silence
        } else {
            dst[i] = g_Ring[g_Tail];
            g_Tail = (g_Tail + 1u) % MIC_RING_BYTES;
            g_Count--;
        }
    }
    KeReleaseSpinLock(&g_Lock, irql);
}

//
// IOCTL handler: copy the pushed PCM into the ring.
//
static VOID
MicPipe_EvtIoDeviceControl(
    _In_ WDFQUEUE   Queue,
    _In_ WDFREQUEST Request,
    _In_ size_t     OutputBufferLength,
    _In_ size_t     InputBufferLength,
    _In_ ULONG      IoControlCode
)
{
    NTSTATUS  status = STATUS_INVALID_DEVICE_REQUEST;
    ULONG_PTR info = 0;

    UNREFERENCED_PARAMETER(Queue);
    UNREFERENCED_PARAMETER(OutputBufferLength);

    if (IoControlCode == IOCTL_LIBREPODS_MIC_WRITE_PCM && InputBufferLength > 0) {
        PVOID  buf = NULL;
        size_t len = 0;
        status = WdfRequestRetrieveInputBuffer(Request, 1, &buf, &len);
        if (NT_SUCCESS(status)) {
            MicPipeWrite(buf, (ULONG)len);
            info = len;
        }
    } else if (IoControlCode == IOCTL_LIBREPODS_MIC_STATUS) {
        PVOID  buf = NULL;
        size_t len = 0;
        status = WdfRequestRetrieveOutputBuffer(Request, sizeof(LONG), &buf, &len);
        if (NT_SUCCESS(status)) {
            *(LONG *)buf = InterlockedCompareExchange(&g_ReadTick, 0, 0);
            info = sizeof(LONG);
        }
    }

    WdfRequestCompleteWithInformation(Request, status, info);
}

NTSTATUS
MicPipeCreateControlDevice(
    _In_ WDFDEVICE Parent
)
{
    NTSTATUS             status;
    PWDFDEVICE_INIT      init = NULL;
    WDFDEVICE            ctl = NULL;
    WDFQUEUE             queue;
    WDF_IO_QUEUE_CONFIG  qCfg;

    // SYSTEM: all, Builtin Admins: RWX, Everyone: RW (so a non-elevated app can
    // open \\.\LibrePodsMic and push audio).
    DECLARE_CONST_UNICODE_STRING(sddl,
        L"D:P(A;;GA;;;SY)(A;;GRGWGX;;;BA)(A;;GRGW;;;WD)");
    DECLARE_CONST_UNICODE_STRING(ntName,  L"\\Device\\LibrePodsMic");
    DECLARE_CONST_UNICODE_STRING(symLink, L"\\DosDevices\\LibrePodsMic");

    // Driver-scoped, created once. Survives PnP device remove/re-add.
    if (g_ControlDevice != NULL) {
        return STATUS_SUCCESS;
    }

    init = WdfControlDeviceInitAllocate(WdfDeviceGetDriver(Parent), &sddl);
    if (init == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    WdfDeviceInitSetDeviceType(init, FILE_DEVICE_UNKNOWN);
    WdfDeviceInitSetIoType(init, WdfDeviceIoBuffered);

    // Single audio source: only one process may feed the mic at a time. Two
    // writers interleaving in the ring sound like static, so refuse a second
    // open (a stuck feeder is freed on its process exit / a driver reload).
    WdfDeviceInitSetExclusive(init, TRUE);

    status = WdfDeviceInitAssignName(init, &ntName);
    if (!NT_SUCCESS(status)) {
        WdfDeviceInitFree(init);
        return status;
    }

    status = WdfDeviceCreate(&init, WDF_NO_OBJECT_ATTRIBUTES, &ctl);
    if (!NT_SUCCESS(status)) {
        WdfDeviceInitFree(init);  // WdfDeviceCreate only consumes init on success
        return status;
    }

    status = WdfDeviceCreateSymbolicLink(ctl, &symLink);
    if (!NT_SUCCESS(status)) {
        WdfObjectDelete(ctl);
        return status;
    }

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&qCfg, WdfIoQueueDispatchParallel);
    qCfg.EvtIoDeviceControl = MicPipe_EvtIoDeviceControl;
    status = WdfIoQueueCreate(ctl, &qCfg, WDF_NO_OBJECT_ATTRIBUTES, &queue);
    if (!NT_SUCCESS(status)) {
        WdfObjectDelete(ctl);
        return status;
    }

    WdfControlFinishInitializing(ctl);
    g_ControlDevice = ctl;
    return STATUS_SUCCESS;
}

EXTERN_C_END
