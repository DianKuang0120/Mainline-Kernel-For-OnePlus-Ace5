.. SPDX-License-Identifier: GPL-2.0

==============
QSEECOM Driver
==============

The QSEECOM driver exposes Qualcomm's QSEE trusted applications, reached over
the command-based QSEECOM interface, through the TEE subsystem.

The implementation reports ``TEE_IMPL_ID_QSEECOM``.

Why this driver
===============

Qualcomm's secure world can be reached through two different interfaces.
QSEECOM is command-based: load an application by name, send it a numbered
command with a request and a response buffer, and service the listener requests
it raises while it works. smcinvoke, served by the QTEE driver (:doc:`qtee`),
is object-based: services are objects carrying operations that return results
and further objects.

They are not two routes to the same thing. A trusted application is built
against one interface or the other, and one that answers commands over QSEECOM
is not reachable as an smcinvoke object. Which driver a system needs is
therefore decided by the applications in its firmware, not by the age of its
SoC. Three cases arise, and this driver is what serves the first two:

Platforms predating smcinvoke
	QSEECOM is the only interface they have.

Platforms offering both
	The common case, and the one the "legacy" label obscures. smcinvoke has
	been available on Qualcomm SoCs for a long time, but applications
	continued to be built against QSEECOM, so a platform that supports
	smcinvoke may still carry firmware that only answers over QSEECOM. The
	SM8250 this driver was developed against is such a platform: it exposes
	both interfaces, and its fingerprint application answers only on
	QSEECOM.

Platforms where smcinvoke has displaced QSEECOM entirely
	Served by the QTEE driver; this one is not needed.

So the driver is not a fallback for old hardware. It is what makes the
QSEECOM-era applications shipped in a platform's firmware reachable at all, and
on many devices those remain the only implementation of the functions they
provide.

Why not extend the QTEE driver
------------------------------

Because the two are different ABIs, and the TEE subsystem carries one backend
per ABI: ``amdtee``, ``optee``, ``qcomtee``, ``qseecom`` and ``tstee``, each
with its own ``TEE_IMPL_ID_*``.

The difference is not cosmetic. QTEE reaches the secure world through two SCM
calls, ``qcom_scm_qtee_invoke_smc()`` and ``qcom_scm_qtee_callback_response()``,
and models everything as objects; its device advertises
``TEE_GEN_CAP_OBJREF``. QSEECOM has six distinct operations -- register and
unregister a listener, look up, load, shut down and send to an application --
addressed to different secure-world owners and services, and models a session as
a named application taking command buffers. It advertises no OBJREF capability
because it has no objects. A ``tee_device`` reports one ``impl_id`` and one set
of capabilities, so the two cannot share one.

What they do share is the SCM plumbing, and that is already shared: both sets of
helpers live in ``drivers/firmware/qcom/qcom_scm.c``.

The two also coexist. A device supporting both exposes ``/dev/tee0`` from this
driver and ``/dev/tee1`` from QTEE, which is only possible as separate
``tee_device`` instances. A client picks by ``impl_id``.

Devices
=======

``/dev/teeX``
	Client device. Open a session against an application and invoke
	commands in it.

``/dev/teeprivX``
	Privileged device. Load applications and register listener services.

Sessions
========

A session is opened by application **name**, not by UUID, because that is what
QSEE matches on. The name is passed NUL-terminated in a memref, and the UUID
field must be zero.

Invoking a command
==================

Commands take an even number of parameters, from two to ten. The first two are
the request and the response, which share one buffer with the response
following the request. Each further pair is an address-patch descriptor and the
buffer it names, up to four of them; the four-parameter form below is the usual
one.

=====  ==============  ===================================================
param  attr            meaning
=====  ==============  ===================================================
0      MEMREF_INPUT    request header
1      MEMREF_OUTPUT   response, following the request
2      VALUE_INPUT     address-patch descriptor
3      MEMREF_INOUT    payload buffer
=====  ==============  ===================================================

QSEE requires the request to contain the *physical* address of the payload
buffer. User space neither knows nor should supply one, so parameter 2 carries
a descriptor instead: ``value.a`` is the offset in the request to write at and
``value.b`` the width, 4 or 8 bytes. The kernel resolves the address of the
following parameter and patches it in. Further pairs may follow for additional
buffers.

Buffers the secure world reads are copied through kernel-only memory, so user
space cannot change them after validation while the secure world is reading
them.

The size of a MEMREF_OUTPUT is **not** updated on return. QSEECOM reports no
response length, so there is nothing to report one from; the size a caller
passes in is the size of the buffer it offered, and how much of it is
meaningful is for the application's own protocol to say. Callers that expect
the TEE convention of a written-length being returned will be disappointed.

That is a narrow guarantee and should not be read as containment. The client
chooses which offsets to name in patch descriptors, so it can leave any value
it likes at any offset it declines to patch, and an application is free to
treat that as an address. The driver validates the patches it is asked to make;
it cannot validate a protocol it does not know.

Loading applications
====================

A load session is opened on the privileged device with a single MEMREF_INPUT
holding the application name. The image is *not* passed in: the driver fetches
``<name>.mdt`` and its ``.bNN`` segments with ``request_firmware()`` and
assembles them itself, so what can be loaded is whatever the firmware search
path holds rather than whatever bytes a process assembles. This is the same
contract amdtee uses.

The name is therefore part of a firmware path and is validated as one: it must
be non-empty, must fit ``QSEECOM_TEE_MAX_APP_NAME``, and must not contain ``/``
or be ``.`` or ``..``.

Assembly is not what ``qcom_mdt_load()`` does. That places each segment at its
``p_paddr`` for a remoteproc carveout, whereas QSEE's ``APP_START`` takes one
contiguous buffer -- the ``.mdt`` followed by the segment payloads in
program-header order -- described by an ``.mdt`` length and a total length.
``qcom_mdt_read_image()`` produces that form, keeping the format knowledge in
``mdt_loader.c`` rather than duplicating a parser here. For real images the
program headers cannot be used as a file layout at all: segments have been
observed declaring ``p_offset`` 0, which overlaps the headers being parsed, and
two segments declaring the same ``p_offset`` with different sizes.

QSEE keeps the name it was loaded under, which is what a later lookup matches.

Application lifetime
--------------------

Nothing owns a loaded application. The driver's registry entry is
reference-counted: resolving an application by name takes a reference, every
session holds one, and the last one released unloads it. There is no explicit
unload, and no session that is special.

This follows amdtee, which reference-counts a TA by handle and unloads it when
the count reaches zero, and the reason to copy it is failure behaviour. QSEE
refuses to load an application that is already resident, and only a reboot
clears one. If a session owned its application, every client that crashed
between load and close would strand it. A reference dropped by the teardown path
cannot be skipped, so a killed process gives its reference back like any other.

That matters because an application typically asks the normal world for its
stored state only during a first initialisation. Re-initialising one means
dropping every reference to it and loading again, which is not possible if a
crash can leave a reference permanently held.

Applications the boot chain loaded are not owned by this driver and are never
unloaded by it.

Listener services
=================

A trusted application asks the normal world to do work on its behalf, file I/O
above all, by raising a listener request. **It blocks until one is answered.**

A listener is registered by opening a session on the privileged device with the
listener id as a VALUE_INPUT and its shared buffer as a MEMREF_INOUT. A
supplicant then loops on ``TEE_IOC_SUPPL_RECV`` and ``TEE_IOC_SUPPL_SEND``.

There is **one supplicant queue per device**, not one per listener. Each
received request reports the listener it belongs to in ``arg.func``, so a
single supplicant can and must serve them all: two processes each registering a
different listener would race on ``TEE_IOC_SUPPL_RECV`` and consume each
other's requests. The first context to receive claims that role, and a receive
or send from any other is refused. Merely opening the device is not claiming
it, because loading an application opens the same device.

``TEE_IOC_SUPPL_RECV`` returns two parameters: a VALUE_INPUT whose first
element identifies the request, and a MEMREF_INOUT naming the listener buffer
to look at. The identifier is an INPUT because the direction is the
supplicant's: the kernel is handing it a value, exactly as OP-TEE does.
``TEE_IOC_SUPPL_SEND`` takes that identifier back as a VALUE_OUTPUT in its
first parameter. The change of attribute is not a typo: parameter
directions on this interface are named from the supplicant's point of view, and
the core only transfers a value that is marked INPUT on the way out to the
supplicant and OUTPUT on the way back in. A supplicant that overran the timeout
returns to find its request abandoned and possibly a new one in the slot;
echoing the
identifier is what lets an answer to the old request be rejected with
``-ESTALE`` instead of being applied to the new one.

If a listener cannot be withdrawn from the secure world, the driver leaks it
deliberately rather than freeing memory TZ can still write to. That keeps a
reference on the shared buffer, which pins the context, so ``rmmod`` afterwards
blocks in ``tee_device_unregister()``. Leaking is still the right trade --
freeing it invites the secure world to write into memory that has been handed
back -- but a module that will not unload is the visible symptom of it.

Anything other than an explicit success is reported to the secure world as a
failure. Claiming success without having filled the request buffer leaves the
application acting on whatever was in it, which has been observed to hang the
secure world until the watchdog fires.

Only one supplicant may be open at a time; a second gets -EBUSY.

A supplicant that stops answering blocks more than itself
=========================================================

Servicing a listener request happens with the QSEECOM call lock held, and every
QSEECOM caller shares that lock -- including in-kernel ones such as
qcom_qseecom_uefisecapp. So a supplicant that stops answering stalls an EFI
variable read for as long as the request takes to time out, and an application
that will not settle can repeat that for several rounds before the driver gives
up on it.

That is inherent to serialising a single secure-world interface, and it is the
reason the supplicant timeout is as short as it is. It is worth knowing before
granting access to the privileged device: the blast radius of a wedged
supplicant is every QSEECOM user on the system, not just the one that
registered the listener.

Known limitations
=================

If the shutdown call fails when an application's last reference goes, the
application stays resident and the driver keeps its registry entry, so it can
still be reached -- but a module unload at that point frees the registry, and
nothing afterwards can name or unload it short of a reboot. QSEE has no
enumerate command that would let a later probe recover it.

Security considerations
=======================

This driver is the normal-world end of the interface trusted applications are
reached through, and it cannot validate what a command means -- it does not
know any application's payload format. Anyone able to open the client device
and open a session can therefore send arbitrary commands to any loaded trusted
application.

That surface is well travelled on Android, which ships the same interface. The
bug can be in a trusted application's own command handler: CVE-2022-48334 is an
integer overflow in ``drm_verify_keys`` and a resulting buffer overflow inside
the Widevine trusted application, reachable by whoever can send it commands.
CVE-2021-1961 is a buffer overflow from an unchecked offset length while
updating a buffer value. The listener path this driver implements has its own
history: CVE-2019-14041 is a buffer overrun while processing a listener's
modified response, because the size is not checked when writing physical
address information into the message buffer, and CVE-2019-14040 is a
use-after-free inside QSEE itself.

The consequences are not confined to the secure world: a trusted application is
trusted by the hardware in ways the kernel is not, so compromising one is a
route back into the kernel rather than a secure-world-only problem. Access to
the device nodes is therefore part of the security boundary rather than a
packaging detail.

The privileged device requires ``CAP_SYS_ADMIN`` to open. It loads code into
the secure world and registers the services trusted applications call back
into, which is not something to gate on file permissions alone; the TEE core
separates it from the client device by minor number and reports
``TEE_GEN_CAP_PRIVILEGED``, but enforces nothing itself.

No such check is made on the client device, because a fingerprint or
attestation daemon has no business running with ``CAP_SYS_ADMIN``. Guarding it
is discretionary and mandatory access control instead: give it to the one
service that needs it rather than leaving a permissive default, and label it in
whatever LSM policy the system uses. Widening it turns a trustlet bug into
local privilege escalation.
