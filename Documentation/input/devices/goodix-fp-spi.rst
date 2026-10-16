Goodix SPI fingerprint sensor driver
------------------------------------

Goodix supports its SPI fingerprint sensors in two modes. In the ordinary one,
REE (Rich Execution Environment) mode, the operating system drives the
sensor over SPI and does the imaging itself. In the alternative TEE (Trusted
Execution Environment) mode, a trusted application inside the SoC's secure
world drives the sensor over an SPI bus assigned to that side, and the
operating system never touches the bus at all.

This driver currently supports TEE mode only, which means it handles only the
parts of the hardware that the secure world does not manage for itself: the
supply, the reset line and the interrupt. This driver owns those three and
nothing else. It never sees an image or a byte of sensor traffic, and it has
no route to the trusted application; reaching that is user space's business,
through whichever TEE driver the platform provides.

However, support for REE mode can be added at a later time; see the
"Limitations" section below.

The driver is adapted from the downstream Android Goodix ``gf_spi`` driver.

Hardware resources
~~~~~~~~~~~~~~~~~~

=================== ==========================================================
resource            why the normal world holds it
=================== ==========================================================
``vdd`` regulator   the sensor is unpowered until something enables it
reset line          the secure world expects the part already out of reset
interrupt           finger events have to reach a userspace client
=================== ==========================================================

The driver also carries the name of the trusted application paired with this
sensor, taken from the optional ``firmware-name`` device tree property. That
pairing is board knowledge rather than a property of the part, so it belongs
in the device tree; the driver only reads it and passes it on.

Reset and power-up timing
~~~~~~~~~~~~~~~~~~~~~~~~~

``gf_hw_reset()`` asserts reset, waits 3 ms, and releases it. A caller may
pass a further settle delay to wait once reset is released.

Both short waits use ``usleep_range()`` rather than ``msleep()``. The reset
pulse is 3 ms and the settle after the supply comes up is 10 ms, and both are
below the roughly 20 ms under which ``msleep()`` rounds up to the timer tick:
a nominal 3 ms sleep can become 4 ms or more depending on ``CONFIG_HZ``, which
lengthens every bring-up without making the pulse any more reliable. The
downstream driver this one is adapted from used ``msleep()`` for both.

The upper bounds passed to ``usleep_range()`` are 1 ms above the lower ones,
which lets the timer subsystem coalesce these sleeps with nearby ones instead
of programming an interrupt for each.

User space interface
~~~~~~~~~~~~~~~~~~~~

**/dev/goodix_fp**, a miscdevice, carries ioctls with magic ``'g'``. Fifteen
are defined, inherited from the downstream ABI. The bring-up sequence a client
actually needs is four of them, in this order and under a single open:

===== ========================= == ==========================================
order ioctl                     nr effect
===== ========================= == ==========================================
1     ``GF_IOC_ENABLE_POWER``   7  enable the regulator
2     ``GF_IOC_RESET``          2  pulse the reset line
3     ``GF_IOC_ENABLE_SPI_CLK`` 5  permit the secure world's SPI traffic
4     ``GF_IOC_ENABLE_IRQ``     3  start delivering finger events
===== ========================= == ==========================================

Closing the descriptor undoes all of it, so a client has to hold the device
open for as long as it wants the sensor alive. The remaining ioctls exist for
the Android HAL's benefit and are not needed to drive the sensor.

**/sys/.../firmware_name** reports the ``firmware-name`` property, so a client
does not have to read the device tree itself. Reading it fails with
``-ENODATA`` when the property is absent.

**A netlink socket**, protocol 25, carries a one-byte event
``GF_NET_EVENT_IRQ`` (1) sent from the interrupt handler. That is the whole
protocol: the message says an interrupt happened and nothing more. What
happened is a question only the trusted application can answer, so a client
learns it by asking the application rather than the driver. Each interrupt
also takes a 2-second wakeup event, so a finger landing on a suspended device
keeps the system up long enough for the client to service it.

Limitations
~~~~~~~~~~~

The netlink protocol number and the ioctl surface are both inherited from the
downstream driver rather than designed here. Protocol 25 is unassigned in
``include/uapi/linux/netlink.h``, which stops at ``NETLINK_SMC`` (22), and a
finger-down event is an input event that the input subsystem could carry
instead. Neither is settled.

REE mode is not implemented. Adding it means porting over the SPI transfer and
image scanning code the downstream driver builds under ``SUPPORT_REE_SPI``,
which the source this one is adapted from does not carry at all.

Only capacitive parts from Goodix's GF range have been tested. The GW
in-display optical range can be served as well: Goodix drive both ranges from
a single downstream source, and the in-display copies differ from the
capacitive ones mostly in board power and pin configuration rather than in
anything the driver itself does.

The downstream display notifier is not carried over. Its role is to wake the
display panel as soon as a finger lands while the screen is off, overlapping
display wake with finger matching to reduce the perceived latency; it does not
serve the fingerprint sensor itself. Reinstating it would mean using
``drm_panel_follower`` rather than the display APIs downstream binds to.
