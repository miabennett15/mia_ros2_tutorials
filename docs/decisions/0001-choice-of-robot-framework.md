---
status: accepted
date: 2023-11-03
deciders: Tim Gentry
---
# Choice of robot framework

## Context and Problem Statement

The key considerations when selecting a robot framework include capabilities, ease of use, scalability, security, and extensibility to customize as needed. Which should we choose?

## Decision Drivers

* Needs to run on the Raspberry Pi (4 and 5) and only the latest, Debian Bookworm based, Raspberry Pi OS supports the Raspberry Pi.
* We need to run Raspberry Pi OS to use `libcamera` to drive both cameras. `libcamera` will not compile on Ubuntu on the RPi 5. 

## Considered Options

* [ROS (v1) "Noetic"](https://wiki.ros.org/noetic)
* [Long Term Support (LTS) ROS2 "Humble"](https://docs.ros.org/en/humble/)
* [Latest ROS2 "Iron"](https://docs.ros.org/en/iron/)
* No framework (bespoke solution)

## Decision Outcome

Chosen option: Latest ROS2 "Iron". While there is a strong argument for the "Humble" Long Term Support (LTS) version of ROS2, there are outstanding fixes for installing on Debian Bookworm in "Iron" that haven't been backported to "Humble" and the process appears completely manual and very slow. Given Debian Bookworm is not supported by "Humble", core contributor motivation to apply these fixes is low.

### Consequences

* Good, because we can run our code on a Raspberry Pi 5 and developer VM.
* Good, because we use the ROS2 community for packages and solutions to problems.
* Neutral, because we need to learn ROS2.
* Bad, because there may we more work needed to update our code in November 2024.

## Pros and Cons of the Options

### ROS (v1) "Noetic"

ROS Noetic Ninjemys is the thirteenth ROS distribution release. It was released on May 23rd, 2020.

Required Support provided for Ubuntu 20.04 (Jammy). Recommended Support for Debian Buster and Fedora 32.

* Good, because of a large variety of ROS (v1) packages (esp. SLAM packages).
* Good, because standard security maintenance support for Ubuntu 20.4 ends in mid 2026.
* Neutral, because Long Term Support for Debian Buster ends in mid 2024.
* Bad, because the vast majority of the ROS development community has moved to ROS2.
* Bad, because support for Ubuntu 20.4 ends in April 2025 and Ubuntu 20.4 does not provide ARM ISOs for Apple silicon Macs.
* Bad, because support for Debian Buster ended in April 2022.

### Long Term Support (LTS) ROS2 "Humble"

Humble Hawksbill is the eighth release of ROS 2. It was release on May 23, 2022.

It is primarily supported on the following platforms: Tier 1: Ubuntu 22.04 and Windows 10; Tier 2: RHEL 8; Tier 3: Ubuntu 20.04, macOS and Debian Bullseye.

* Good, because it is the Long Term Support version of ROS2, supported until May 2027.
* Good, because standard security maintenance support for Ubuntu 22.4 ends in mid 2026 with ARM ISOs for Apple silicon Macs.
* Good, because Long Term Support for Debian Bullseye ends in mid 2026.
* Neutral, because less variety of pre-built ROS2 packages (esp. SLAM packages).
* Bad, because Raspberry Pi 5 does not support Debian Bullseye.
* Bad, because fixes for Debian Bookworm (which Raspberry Pi 5 does support) are extremely slow to be backported.

### Latest ROS2 "Iron"

Iron Irwini is the ninth release of ROS 2. It was release on May 23, 2023.

It is primarily supported on the following platforms: Tier 1: Ubuntu 22.04 and Windows 10; Tier 2 platform: RHEL 9; Tier 3: macOS and Debian Bullseye.

* Good, because standard security maintenance support for Ubuntu 22.04 ends in mid 2026 with ARM ISOs for Apple silicon Macs.
* Good, because Long Term Support for Debian Bullseye ends in mid 2026.
* Good, because fixes for Debian Bookworm (which Raspberry Pi 5 does support) are quick to be included.
* Good, because Long Term Support for Debian Bookworm has yet to be confirmed (estimate mid 2028).
* Neutral, because it is not an LTS version of ROS2, but is supported until November 2024.
* Neutral, because less variety of pre-built ROS2 packages (esp. SLAM packages).
* Bad, because {argument d}

### No framework (bespoke solution)

We could use no framework and build a completely bespoke robot.

* Good, because we have no framework or operating system dependencies.
* Bad, because we have to spend a lot of time building all the utility components that we get for free from an existing framework.
* Bad, because we cannot use off-the-shelf components for our LiDAR, IMU, etc.

## More Information

Evidence to support this decision was gathered from:

* https://www.ros.org/reps/rep-2000.html
* https://www.raspberrypi.com/software/operating-systems/
* https://ubuntu.com/about/release-cycle
* https://wiki.debian.org/LTS
