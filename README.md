# Formal methods for Blinky Blocks (PRISM models)

This repository contains PRISM models for an energy-aware / self-adaptive Blinky Blocks scenario.
It includes:
- a **single-module** model (baseline),
- a **two-cluster** model (distributed / clustered behavior),
- an  empirical instrumentation report
- a shared set of **properties** that can be checked on both models (same labels).
- scenario excerpt for the parameter extraction.

The goal of this artifact is to show how a self-adaptive system can be modelled and analysed with a conditional probabilistic, state guarded transition system and intervening controlling modules.

**Authors:** Antonios Naguib, Olga Kouchnarenko, Frédéric Lassabe

## Repository structure

- `SingleBB Model/`
  - PRISM model of a single Blinky Block scenario.
  - Folder-specific README explains the model and how to run it.

- `2 Cluster model/`
  - PRISM model with 2 clusters of BBs.
  - Folder-specific README explains the model and how to run it.

- `General_properties.pctl`
  - Properties intended to be runnable on both models (shared labels / shared semantics).

## Requirements

- PRISM model checker (recommended: PRISM 4.7+)
[PRISM download](https://www.prismmodelchecker.org/download.php)

Optional:
- Java (if required by your PRISM distribution)
- Graphviz (only if you export graphs)

## Quick start (build model)

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
## Replicate the prism experiments
The files for replicating the PRISM experiments for the models can be found in the folder singleBB model as Single_BB.pm and folder 2 clusters model as 2clusters_Fullmodel.pm. The necessary property file, containing the properties used for the experiments, is General_properties.pctl in the same folder. The latter properties could be applied for both models.

Open the PRISM GUI by opening the executable xprism that should have been downloaded when you downloaded PRISM. Open one of the model files by going to Model -> Open model and selecting file with the extension of **".pm"** Parse and build the model by pressing F2 and F3, respectively. Prism will ask about the scenario constants for both models. A user might import constants stated at the top of the model, or to define their own constants to discover the model more.

To load the properties, go to the Properties Tab in the lower left corner. Open the properties list by going to Properties -> Open properties list and select General_properties.pctl. The GUI should now look like the following.
<img width="1028" height="750" alt="image" src="https://github.com/user-attachments/assets/152bac67-57eb-4ef6-ba30-101851210139" />

The experiments will use a variable named k for the number of time steps. To declare this variable, make a double click in the Constants field and change the name from C0 to k.

To run an experiment, click one of the properties and press F7. In the dialog that opens, first decide which range your parameter should have, i.e., how many time steps you want to consider; in the paper we display the graph with 10 time steps. Click on Okay, give the graph a name and either print it to an already existing graph or to a new one.

It is also possible to inspect the values that were calculated for the graph. To do that, in the Experiments part of xprism, do a right click on the property whose results you want to inspect and click on View results as shown in the picture below.

<img width="1025" height="452" alt="image" src="https://github.com/user-attachments/assets/1471b097-28cf-468c-aed6-810ed508e8bb" />
This will enable you to determine after how many time steps the probability for the respective property to be satisfied is above a certain threshold.

For more information about PRISM experiments, including how to run them from the command line, consult the [PRISM manual.]([https://www.prismmodelchecker.org/download.php](https://www.prismmodelchecker.org/manual/RunningPRISM/Experiments))

## Extend and modify the artifact

The artifact can be modified and extended in different ways, some ideas are collected below.

- Explore new scenarios.
- Analyse different properties.
- Change the probabilites of the transitions.
- Include new modules. This would probably also entail modifying or extending the synchronisation between the different modules and extending the task_controller and environment modules.
- 

## Acknowledgments
The authors are supported by the ANR ADAPT (grant number ANR-23-CE25-0004) and the EIPHI Graduate School (grant number ANR-17-EURE-0002).
