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
├── Firmware/     # Embedded firmware (ESP32)
├── Software/     # Control code
├── assets/       # Images and media
└── README.md
```
---

## Reference

This repository accompanies the paper:

**An Untethered Bioinspired Robotic Tensegrity Dolphin with Multi-Flexibility Design for Aquatic Locomotion**  
Luyang Zhao, Yitao Jiang, Chun-Yi She, Mingi Jeong, Haibo Dong, Alberto Quattrini Li, Muhao Chen, and Devin Balkcom  
*IEEE International Conference on Soft Robotics (RoboSoft), 2025*

Paper link:  
https://ieeexplore.ieee.org/document/11020959

If you use this repository in your research, please cite the above paper.

---

## Fabrication and Reproduction

This repository provides the design files and code necessary to reproduce the robotic system at a system level.  
Detailed fabrication procedures, material choices, and experimental setup are described in the accompanying paper.

In brief, reproduction involves:
1. Fabrication of the rigid head components and tensegrity tail skeleton
2. Molding of the silicone tail using the provided mold designs
3. Assembly of the actuation system and connectors
4. Integration of electronics and firmware
5. Waterproofing, tuning, and aquatic testing

Readers are encouraged to refer to the paper for detailed descriptions of materials, dimensions, and experimental protocols.

---

## Waterproofing Considerations

Waterproofing is a critical aspect of the robotic system and requires careful mechanical and material design.  
Based on our implementation, the key considerations include:

1. **Shaft sealing**  
   Proper sealing of rotating shafts is the primary and most critical waterproofing measure.  
   Inadequate shaft sealing cannot be compensated for by external coatings or adhesives.

2. **Silicone-based sealing and bonding**  
   Silicone glue and sealants are used to seal housing interfaces, forming the system-level waterproof barrier.

3. **Water-resistant fabrication materials**  
   Waterproof or water-resistant printing materials (e.g., resin-based prints) help reduce water ingress but should be considered a supporting measure rather than a standalone solution.

Successful waterproofing requires the **combined use** of all three approaches rather than reliance on any single method.

