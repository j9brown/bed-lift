# Bed-lift controller

Controls a motorized bed-lift with two synchronized linear actuators for vertical lifting and lowering and two pairs of synchronized stepper motors for horizontal extension and retraction.  It fits in the back of a campervan.

## Circuit board revisions

Refer to the following documents for design synopsis, schematics, and circuit board illustrations.

- [v1](./hardware/v1/design-and-errata.md): Fixed errata, improved assignment of pin functions, added span limit hall sensors, buzzer and ESD protection
- [v0.2](./hardware/v0.2/design-and-errata.md): First feature-complete circuit board
- [v0.1](./hardware/v0.1/design-and-errata.md): Initial hand-assembled prototype

## About the "forklift" bed

This custom motorized bed fits in the back of a small campervan to allow dual-use of the living area in a standard wheelbase vehicle.  In its raised position, the bed platform is only supported from the back which gives it the appearance of a pallet on a forklift, hence the name.

To maximize space efficiency, the bed is designed for sleeping crosswise with the head and feet towards the sides of the vehicle.  This choice of sleeping orientation poses unique engineering challenges; it is more common to design lifting beds for sleeping lengthwise with the head and feet towards the front and back of the vehicle.

Because the van is narrower at roof elevantion than at sleeping elevation, the bed extends and retracts to span the gap between the main body of the bed platform and the sides of the vehicle.  The parts that extend and retract are called the bed wings (aka. wing spans).  The bed mattress is made of three cushions: one large central cushion on the main body of the bed platform and two narrower cushions that rest on the two bed wings.  The narrower cushions are removed and stowed on top of the bed before retracting the bed wings.

- When the bed is raised, the bed wings retract into the bed platform and the platform is cantilevered from steel columns attached to the rear doorframe and floor of the vehicle.

- When the bed is lowered, the bed wings extend from the bed platform and rest upon reinforced windowsills to transfer the load of the occupants to the vehicle walls.

The bed has limited load bearing capacity while raised so it must be lowered when occupied or while driving to prevent excess torque through the cantilever.  As an advantage, there are no cables, straps, bulkheads, guides, posts, or other obstructions in the front of the living area to support the bed.  As a disadvantage, the bed structure is heavy and difficult to build.

Feel free to contact me for advice if you are interested in building a bed like mine (and be prepared for a challenge)!

[Video tour of the v0.2 prototype](https://vimeo.com/1154513482/b7c7d3c8f7)

<details>
<summary>Photos of v0.2 prototype</summary>

### Bed lift in lowered position viewed from outside
![Bed lift in lowered position viewed from outside](./docs/v0.2/outside-lowered.jpg)

### Bed lift in raised position viewed from outside
![Bed lift in raised position viewed from outside](./docs/v0.2/outside-raised.jpg)

### Bed lift in lowered position viewed from inside
![Bed lift in lowered position viewed from inside](./docs/v0.2/inside-lowered.jpg)

### Bed lift in raised position viewed from inside
![Bed lift in raised position viewed from inside](./docs/v0.2/inside-raised.jpg)

### Bed lift controls
![Bed lift controls](./docs/v0.2/controls.jpg)

### Bed lift driver
![Bed lift driver](./docs/v0.2/driver.jpg)

### Bed lift position encoder
![Bed lift driver](./docs/v0.2/lift-limit.jpg)

</details>

<details>
<summary>Photos of v0.1 prototype</summary>

### Bed lift in lowered position
![Bed lift in lowered position](./docs/v0.1/bed-lift-prototype-lowered.jpg)

### Bed lift in raised position
![Bed lift in raised position](./docs/v0.1/bed-lift-prototype-raised.jpg)

</details>

<details>
<summary>CAD model</summary>

### Bed lift in lowered position
![Bed lift in lowered position](./docs/model/bed-lift-model-lowered.png)

### Bed lift in raised position
![Bed lift in raised position](./docs/model/bed-lift-model-raised.png)

</details>

## How to build the firmware

The firmware is based on Zephyr RTOS v4.3 and compiled with `west`.

1. Install the Zephyr RTOS SDK tools.  You can either follow the [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html) or install an extension for your IDE, such as [Zephyr IDE](https://zephyr-ide.mylonics.com/) or [Workbench for Zephyr](https://z-workbench.com/), that can install the tools for you.

2. Install the [xPack OpenOCD](https://xpack-dev-tools.github.io/openocd-xpack/docs/install/) package for your OS and add it to your system `PATH`.  This step is needed because the version of OpenOCD that is distributed with Zephyr is too old and doesn't support the STM32C0x series of chips.  We need OpenOCD v0.12.0-7 at minimum.

3. Get the code and its dependencies.  This is a little more complicated than simply cloning this git repository because the Zephyr `west` multi-tool assumes that all of the code resides within a top directory called a workspace.  You will need to create an empty directory to be the workspace first.  Here's how to do it from the command-line and the same thing can be done with IDE extensions.

```sh
$ mkdir bed-lift-workspace
$ cd bed-lift-workspace
$ west init -m https://github.com/j9brown/bed-lift
$ west update
```

4. Build and flash the firmware.

```sh
$ cd bed-lift/firmware/app
$ west build -p
$ west flash
```

Firmware for the older prototypes is preserved in the git history.

- The v0.2 firmware was based on Zephyr RTOS v4.2.1 and compiled with Platform IO.
- The v0.1 firmware was based on Arduino and compiled with Platform IO.

## Notice

The bed-lift software, documentation, design, and all copyright protected artifacts are released under the terms of the [MIT license](LICENSE).

The bed-lift hardware is released under the terms of the [CERN-OHL-W license](hardware/LICENSE).
