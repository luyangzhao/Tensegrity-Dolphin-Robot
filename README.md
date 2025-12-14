# Tensegrity Dolphin Robot

Open-source design files for an **untethered bio-inspired robotic dolphin** with a **tensegrity-based, multi-flexibility tail** for efficient aquatic locomotion.

This repository provides the mechanical, electronic, and software resources needed to understand, reproduce, and extend the robotic system presented in our RoboSoft 2025 paper.

---

## Overview

The Tensegrity Dolphin Robot features a **hybrid rigid–soft architecture**:
- A rigid, waterproof head module housing electronics, actuation, and power.
- A flexible tensegrity tail skeleton enabling distributed compliance.
- A molded silicone tail skin providing smooth hydrodynamic surfaces.
- Modular connectors allowing rapid tail replacement and reconfiguration.

By varying the internal tail skeleton geometry, the robot can achieve different flexibility profiles and swimming performance, enabling systematic studies of morphology–locomotion trade-offs.

---

## Repository Structure

```
Tensegrity-Dolphin-Robot/
├── CAD/          # Mechanical design files (head, tail, molds, connectors)
├── PCB/          # Electronics design files
├── firmware/     # Embedded firmware (ESP32)
├── software/     # Control code
├── assets/       # Images and media
└── README.md
```

### Key Directories

- **`CAD/`**  
  Mechanical design files, including:
  - Rigid head enclosure
  - Tensegrity tail skeleton variants
  - Silicone tail molding tools
  - Modular connectors  
  Each subdirectory contains its own README with detailed descriptions.

- **`PCB/`**  
  Schematics, layout files, and bills of materials for the onboard electronics.

- **`firmware/`**  
  Embedded firmware for motor control, sensing, and wireless communication.

- **`software/`**  
  High-level control

- **`docs/`**  
  Build guides, assembly notes, and fabrication considerations.

---

## Reference

This repository accompanies the paper:

**An Untethered Bioinspired Robotic Tensegrity Dolphin with Multi-Flexibility Design for Aquatic Locomotion**  
Luyang Zhao, Yitao Jiang, Chun-Yi She, Mingi Jeong, Haibo Dong, Alberto Quattrini Li, Muhao Chen, and Devin Balkcom  
*IEEE International Conference on Soft Robotics (RoboSoft), 2025*

Paper link:  
https://ieeexplore.ieee.org/document/11020959

If you use this repository in your research, please cite the above paper.

