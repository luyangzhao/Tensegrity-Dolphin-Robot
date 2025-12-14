# Firmware – Embedded Control

This directory contains the **embedded firmware** used to operate the **Tensegrity Dolphin Robot**.

The firmware runs on an onboard microcontroller and is responsible for low-level motor actuation, communication, and system management for untethered aquatic operation.

---

## Overview

The firmware supports:
- Cable-driven motor control for tail actuation
- Wireless communication for command and monitoring
- Basic system management for untethered operation

The firmware is designed to interface directly with the PCB referenced in `PCB/` and the mechanical components defined in `CAD/`.
