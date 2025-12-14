# Tensegrity Tail – Skeleton Variants

This directory contains **six tensegrity tail skeleton variants** corresponding to the designs evaluated in the paper.  
All variants share the same overall geometry and interface to the head module, but differ in **rib height ratio (h₁:h₂)** and **structural thickness ratio**, resulting in different tail flexibility and swimming performance.

---

## Skeleton Variants

The following STL files correspond to the skeleton types reported in the paper:

| Skeleton Type | h₁:h₂ | Thickness Ratio | STL File |
|--------------|------|-----------------|----------|
| Type 1 | 1:1 | 1:1 | `Type1_h1-1_thick1-1.stl` |
| Type 2 | 1:1 | 2:1 | `Type2_h1-1_thick2-1.stl` |
| Type 3 | 1:1 | 3:1 | `Type3_h1-1_thick3-1.stl` |
| Type 4 | 1:2 | 1:1 | `Type4_h1-2_thick1-1.stl` |
| Type 5 | 1:2 | 2:1 | `Type5_h1-2_thick2-1.stl` |
| Type 6 | 1:2 | 3:1 | `Type6_h1-2_thick3-1.stl` |

---

## Usage Notes

- All skeleton variants are **mechanically compatible** with the same head module, connectors, and actuation system.
- Skeletons can be swapped without modifying electronics or control software.
- Changing the skeleton alters tail flexibility, bending amplitude, swimming speed, and energy efficiency.

Type 4 (`Type4_h1-2_thick1-1.stl`) has fastest swimming speed and lowest cost of transport.
