# Software – High-Level Control

This directory contains **high-level control and interaction software** for the **Tensegrity Dolphin Robot**.

The software runs on an external computer and communicates wirelessly with the onboard firmware to command tail actuation and support basic experimentation.

---

## Contents

- `Control_Program_Dolphin.ipynb`  
  A Jupyter Notebook providing a high-level control interface for commanding the two tail motors to generate **undulatory swimming motions**.

The notebook is intended for demonstration, testing, and exploratory control, rather than optimized autonomous operation.

---

## Notes

- The software communicates with the embedded firmware provided in `firmware/`.
- Control parameters and motion patterns may require tuning depending on the tail configuration and materials.
