# Formal methods for Blinky Blocks (PRISM models)

This repository contains PRISM models for an energy-aware / self-adaptive Blinky Blocks scenario.
It includes:
- a **single-module** model (baseline),
- a **two-cluster** model (distributed / clustered behavior),
- a shared set of **properties** that can be checked on both models (same labels).

## Repository structure

- `SingleBB Model/`
  - PRISM model of a single Blinky Block scenario.
  - Folder-specific README explains how to run it.

- `2 Cluster model/`
  - PRISM model with 2 clusters of BBs.
  - Folder-specific README explains how to run it.

- `General_properties.pctl`
  - Properties intended to be runnable on both models (shared labels / shared semantics).

## Requirements

- PRISM model checker (recommended: PRISM 4.7+)

Optional:
- Java (if required by your PRISM distribution)
- Graphviz (only if you export graphs)

## Quick start (run properties)

From the repository root:

### 1) Single BB model
```bash
prism "SingleBB Model/single_BB.pm" General_properties.pctl

