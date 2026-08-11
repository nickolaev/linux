.. SPDX-License-Identifier: GPL-2.0

===================================
Multikernel Kernfs Interface Usage
===================================

Overview
========

Multikernel is managed through a kernfs filesystem::

    mount -t multikernel none /sys/fs/multikernel

Every kernel, host or spawn, mounts the same interface: a kernel that was
given a baseline manages a pool and can create instances; a spawn kernel
sees its own instance under ``instances/`` and can modify itself.

Layout
======

::

    /sys/fs/multikernel/
    ├── device_tree            # Write: the baseline. Read: the pool tree
    ├── instances/
    │   └── <name>/
    │       ├── id             # Instance id (read-only)
    │       ├── status         # empty | ready | loaded | active | failed
    │       └── device_tree    # The instance tree (read-only, DTB)
    └── overlays/
        ├── new                # Write an overlay DTBO here
        └── tx_<N>/            # One directory per transaction

The trees are described in ``device-tree.rst`` and the overlay format in
``overlays.rst``.

Workflow
========

1. **Write the baseline** once, to give the pool its CPUs, memory and
   devices::

       cat baseline.dtb > /sys/fs/multikernel/device_tree

   Afterwards the pool is changed through overlays targeting
   ``/resources``; a second baseline is refused while the pool is in use.

2. **Create an instance** with an ``instance-create`` overlay, and hand it
   resources with ``cpu-add``, ``memory-add`` and ``device-add`` fragments
   targeting ``/instances/<name>``::

       cat create.dtbo > /sys/fs/multikernel/overlays/new
       cat /sys/fs/multikernel/instances/web/status
       # ready

3. **Load and start a kernel** in it with ``kexec_file_load()`` addressed
   at the instance id (``kerf load`` and ``kerf exec`` do this). The
   instance's tree, with a ``/chosen`` node for the boot handoff, becomes
   the spawn's boot device tree.

4. **Read back** what an instance owns at any time::

       dtc -I dtb -O dts /sys/fs/multikernel/instances/web/device_tree

5. **Take resources back** with ``cpu-remove``, ``memory-remove`` and
   ``device-remove``, and destroy the instance with ``instance-remove``
   once it is stopped.

SR-IOV Assignment Boundary
===========================

SR-IOV assignment is intended for cooperative spawned kernels. Filtering
configuration-space access and enumerating only assigned BDFs prevents
accidental access; it is not a security boundary against a privileged kernel
that deliberately issues configuration cycles or maps host physical windows.

The host keeps exclusive leases for assigned VFs. Assignment fails unless the
device is an SR-IOV VF whose identity and ownership can be validated. Host
drivers remain bound while a VF is merely in the pool and are detached only
while committing a lease. Nested kernels cannot establish this host-owned
lifecycle and therefore cannot lease PCI devices to their children.

Instance States
===============

- **empty**: the instance exists but holds no resources yet
- **ready**: resources reserved, a kernel can be loaded
- **loaded**: a kernel is loaded and can be started
- **active**: the kernel is running
- **failed**: an error occurred; check ``dmesg``

Restrictions
============

- Instances are created and destroyed only through overlays; ``mkdir``
  under ``instances/`` is not supported.
- Instance files are read-only; an instance's resources change through
  overlays targeting ``/instances/<name>``.
- Rollback (``rmdir`` on a transaction) cannot destroy a running instance.
- Logical CPU 0 handles assigned-device MSI forwarding and must remain online
  while those devices are active.
- Assigned MSI events are recorded in per-instance shared mailboxes before a
  doorbell is sent. Masked events remain pending and lost doorbells are retried;
  each spawn drains only its own parent-link mailbox.
