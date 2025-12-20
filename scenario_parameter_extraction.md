# Empirical Calibration of Probabilistic Parameters

This repository contains some empirical scenarios used to **calibrate the probabilistic
parameters** of the Blinky Blocks (BB) MDP model presented in the paper
*“Module-based Modeling and Assessment of Modular Robots”*.

The goal of these experiments is **not** to exhaustively model every possible execution,
but to extract **reliable coefficients and ratios** that ground the formal model in
real measurements.

---

## Overview

The PRISM model uses a small set of calibrated parameters to represent:

- colour-dependent actuation reliability,
- distance-dependent degradation across clusters,
- topology effects (line vs rectangle),
- execution mode effects (sequential vs parallel),
- energy-aware fallback behavior.

These parameters are derived from **controlled experimental scenarios** executed on a
real BB platform.

Videos of the experiments are available upon request.

---

## Experimental Setup (Common to All Scenarios)

- **Number of blocks:** 30
- **Clusters:**  
  - Cluster 1 (c1): 15 BBs  
  - Cluster 2 (c2): 15 BBs
- **Power supply:** single 5V / 2A source
- **Between tasks:** LEDs set to zero for 1 second
- **Metric:** a failure is recorded when at least one BB does not exhibit the intended output

---

## Scenarios Used for Calibration

### Scenario 14 
NO.OF BLOCKS:30
Pattern:c1(line/15)--c2(line/15)
Operator: sequential execution for ALL tasks (62s/task)
trials:5
No.of tasks: 3
T1:c1(B_h)--c2(W_h)
T2:c1(B_h)--c2(W_m)
T3:c1(B_h)--c2(W_l)

 Notes:
 -Trial1: T1(6failures at last c2 white),T2(1 failure in white id29 @c2 ),T3(no failures) 
 -Trial2: T1(6failures at last c2 white),T2(1 failure in white id29 @c2 ),T3(no failures)
 -Trial3: T1(6failures at last c2 white),T2(1 failure in white id30 @c2 ),T3(no failures)
 -Trial4: T1(6failures at last c2 white),T2(1 failure in white id26 @c2 ),T3(no failures)
 -Trial5: T1(6failures at last c2 white),T2(2 failure in white  @c2 ),T3(no failures) 
 -Between each task and another, I give LED values to "0" for 1s.
 
 ---
### Scenario 15 
NO.OF BLOCKS:30
Pattern:c1(line/15)--c2(line/15)
Operator: parallel execution for ALL tasks (20s/task)
trials:5
No.of tasks: 3
T1:c1(B_h)--c2(W_h)
T2:c1(B_h)--c2(W_m)
T3:c1(B_h)--c2(W_l)

 Notes:
 -Trial1: T1(7failures at last c2 white),T2(1 failure in white id29 @c2 ),T3(no failures) 
 -Trial2: T1(7failures at last c2 white),T2(1 failure in white id29 @c2 ),T3(no failures)
 -Trial3: T1(6failures at last c2 white),T2(no failures),T3(no failures)
 -Trial4: T1(6failures at last c2 white),T2(1 failure in white id29 @c2 ),T3(no failures)
 -Trial5: T1(6failures at last c2 white),T2(1 failure in white  @c2 ),T3(no failures) 
 -Between each task and another, I give LED values to "0" for 1s.
 
 ---
### Scenario 16 
NO.OF BLOCKS:30
Pattern:c1(line/15)--c2(rec/3*5)
Operator: parallel execution for ALL tasks (62s/task)
trials:5
No.of tasks: 3
T1:c1(B_h)--c2(W_h)
T2:c1(B_h)--c2(W_m)
T3:c1(B_h)--c2(W_l)

 Notes:
 -Trial1,2,3,5: T1(3failures at c2 white),T2(no failures ),T3(no failures) 
 -Trial4: T1(4failures at last c2 white),T2(no failures),T3(no failures)
 -Between each task and another, I give LED values to "0" for 1s.
 
---
### Scenario 17 
NO.OF BLOCKS:30
Pattern:c1(line/15)--c2(rec/3*5)
Operator: sequential execution for ALL tasks (62s/task)
trials:5
No.of tasks: 3
T1:c1(B_h)--c2(W_h)
T2:c1(B_h)--c2(W_m)
T3:c1(B_h)--c2(W_l)

 Notes:
 -Trial1,2,3,4,5: T1(2failures at c2 white),T2(no failures ),T3(no failures) 
 -Between each task and another, I give LED values to "0" for 1s.
 

---

## Parameter Extraction Strategy

Rather than fitting exact probabilities per scenario, we adopt a **ratio-based calibration**:

1. **Base success probabilities**  
   Derived from line–line, high-intensity tasks (Scenarios 14–15).

2. **Distance scaling (`α₂`)**  
   Computed as the ratio between far-cluster and near-cluster failure rates,
   preserving colour ordering while avoiding perfect reliability.

3. **Topology scaling (`θ`)**  
   Estimated by comparing line–line and line–rectangle executions.

4. **Execution-mode scaling (`γ`)**  
   Estimated by comparing sequential and parallel executions under identical tasks.

5. **Fallback coefficients (`β₁`, `β₂`)**  
   Derived from the observed disappearance of failures when transitioning from
   high to medium or low intensity.

This approach yields **conservative, physically meaningful parameters** that generalise
beyond the exact experimental setup.

---

## Relation to the PRISM Model


Only **Scenario 14–17** are used for calibration. All verification results reported
in the paper correspond to the calibrated parameter set derived from these scenarios.

---

## Videos and Raw Logs

- Videos documenting the experiments are available upon request.
- Raw per-block logs are intentionally not included to keep the artifact lightweight;
  all derived coefficients and assumptions are explicitly documented.

---

## Reproducibility Notes

- The provided PRISM model can be reparameterised using the constants extracted here.
- The scenarios are sufficient to reproduce:
  - colour-dependent failure ordering,
  - motif effects,
  - execution-mode effects,
  - energy-aware fallback behavior.

---

## Contact

For additional data, videos, or clarification regarding the calibration process,
please contact the authors.
