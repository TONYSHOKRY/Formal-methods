# Formal methods for Blinky Blocks (PRISM models)

This repository contains PRISM models for an energy-aware / self-adaptive Blinky Blocks scenario.
It includes:
- a **single-module** model (baseline),
- a **two-cluster** model (distributed / clustered behavior),
- a shared set of **properties** that can be checked on both models (same labels).

The goal of this artifact is to show how a self-adaptive system can be modelled and analysed with a conditional probabilistic, feature guarded transition system and a controller switching between features.

**Authors:** Antonios Naguib, Olga Kouchnarenko, Frédéric Lassabe

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
[PRISM download]([https://example.com](https://www.prismmodelchecker.org/download.php))

Optional:
- Java (if required by your PRISM distribution)
- Graphviz (only if you export graphs)

## Quick start (run properties)

From the repository root:

### 1) Single BB model
```bash
prism "SingleBB Model/single_BB.pm" General_properties.pctl
```





## Read the out.log File
```bash
PRISM
=====

Version: 4.8.1
Date: xxx
Hostname: xxx
Memory limits: cudd=1g, java(heap)=1g

Parsing model...

Type:        MDP
Modules:     Operator pattern Hardware Environment Task_Controller c1 c2
Variables:   staggered Rec_motif1 Rec_motif2 hw_state sound move env_state1 env_state2 task_counter active1 active2 turn s1 color1 s2 color2
---------------------------------------------------------------------

Building model...
Model constants: theta=1.057,beta1=1.154,beta2=1.154,gamma=0.9725,alpha2=0.9,q_w_high_c1=0.62,q_b_high_c1=0.9,q_r_high_c1=0.9,q_high_pitch_c1=0.8

Computing reachable states...

Reachability (BFS): 23 iterations in 0.01 seconds (average 0.000435, setup 0.00)

Time for model construction: 0.198 seconds.

Warning: Deadlocks detected and fixed in 588 states

Type:        MDP
States:      284576 (1 initial)
Transitions: 1315772
Choices:     946036

Transition matrix: 47704 nodes (1578 terminal), 1315772 minterms, vars: 30r/30c/34nd
```
