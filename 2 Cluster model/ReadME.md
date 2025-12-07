# 2-Cluster Energy-Aware MDP for Blinky Blocks

This folder contains the **full Markov Decision Process (MDP) model** of **two Blinky Blocks clusters** executing tasks under:
- different **motifs** (line vs rectangle),
- different **execution modes** (synchronous vs staggered),
- and **environment / hardware feedback** (sensing + current sensor).

The model is written for **PRISM** (and can be ported to STORM).

---

## Files in this folder

- `two_clusters_full_model.prism`  
  Main MDP model with:
  - cluster modules `c1` and `c2`,
  - `Task_Controller`,
  - `Environment`,
  - `Hardware`,
  - `Operator` (staggered vs synchronous),
  - `pattern` (line vs rectangle motif flags).

## What this model represents

### Clusters as macro-agents

- `module c1` and `module c2` each represent a **cluster of N1 / N2 Blinky Blocks** (e.g., 10 + 10).
- Internally, each cluster has:
  - **task-level states**: `idle`, `start_task`, `finish`, `task_failed`;
  - **actuation states**:
    - color selection: `choose_Red`, `choose_Blue`, `choose_white`,
    - pitch levels: `high_pitch`, `med_pitch`, `low_pitch`,
    - LED intensity: `high_Led`, `med_Led`, `low_Led`,
    - combined actuation: `high_blink_high_buzz`, `high_blink_med_buzz`,
      `med_blink_high_buzz`, `med_blink_med_buzz`;
  - **sensing states**: `listen_on_moving_on`, `listen_on_moving_off`,
    `listen_off_moving_on`.

So each cluster is a **macro-state machine** aggregating the detailed per-block behavior.

---

### Coordination & scheduling

- `Task_Controller`:
  - manages the **task counter** (`task_counter`),
  - activates/deactivates clusters (`active1`, `active2`),
  - controls **turn-taking** through `turn` when staggered execution is used.
- `Operator`:
  - binary flag `staggered` to switch between:
    - **synchronous execution** (`[step]` transitions),
    - **staggered execution** (`[stag1]` transitions) where clusters are desynchronized.

The combination of `[step]` / `[stag1]` labels + `turn` allows us to study:
- **synchronous multi-cluster behavior** vs
- **staggered / interleaved behavior** under the same abstract task logic.

---

### Environment & hardware feedback

- `Hardware` module:
  - `hw_state ∈ {0,1,2}` represents **current feedback levels**:
    - `0` = nominal,
    - `1` = medium load,
    - `2` = high load.
  - Hardware feedback **forces deterministic fallback** from high → med → low consumption states when current is too high.

- `Environment` module:
  - `env_state1`, `env_state2` encode whether a cluster senses:
    - `0` = idle, `1` = MIC, `2` = ACC, `3` = BOTH.
  - Boolean flags `sound` and `move` model **external activity** (noise or movement).
  - These states influence probabilistic transitions from sensing and actuation states.

Together, these two modules encode a **self-adaptive CPS flavor**:  
decisions in `c1` / `c2` are conditioned on **both** current consumption and environment sensing.

---

### Motif / pattern modeling

- `pattern` module:
  - `Rec_motif1`, `Rec_motif2` flags indicate whether the cluster is running in:
    - **line pattern** (supply at one edge) – base case,
    - **rectangle pattern** – modeled via coefficient `theta`.
- `theta` scales the success probabilities when switching from line → rectangle:
  - rectangle can sustain more blocks under the same power supply,
  - so success probabilities are **boosted** by `theta` in the rectangle case.

---

## Abstractions & design choices

This model is **not** a 1:1 replica of every BB and every packet. Instead:

- **Per-block details are abstracted**:
  - We use **macro-states** (e.g., `high_blink_high_buzz`) instead of
    separate LED + buzzer submachines per block.
  - Communication / spanning tree and low-level protocol (Layer 2/3 packets)
    are *not* explicitly modeled. They are implicitly captured by:
    - success/failure probabilities,
    - environment/hardware guards.

- **Environment is discretized**:
  - MIC / ACC sensing is captured as simple modes (`MIC`, `ACC`, `BOTH`, `idle`),
  - no continuous thresholds, just **qualitative influence** on transitions.

- **Hardware feedback is coarse-grained**:
  - Current sensor is reduced to three levels (0 / 1 / 2),
  - used mainly to **force energy-aware fallback** decisions.

- **Cluster size & distance**:
  - `N1`, `N2` store cluster sizes.
  - Coefficient `alpha2` is planned to capture **increased failure** in cluster 2 due to distance and power limitations (see “Future work” below).

These abstractions keep the model **tractable** while still reflecting:
- topology (via `theta`),
- concurrency / staggering (via `gamma` and `staggered`),
- hardware constraints (via `hw_state`),
- environment (via `env_state*`, `sound`, `move`).

---

## Implicit conditional probabilities

The model uses **conditional probabilities** extensively, but without introducing an explicit “conditional probability syntax”. Instead, they are encoded through:

1. **Base success probabilities**  
   - `q_w_high_c1`, `q_b_high_c1`, `q_r_high_c1`  
   - `q_high_pitch_c1`  
   These represent:
   - *P(success | high LED color X, line motif, non-staggered, nominal current)*,  
   - or *P(success | high pitch, line, nominal current)*, etc.

2. **Fallback coefficients**  
   - `beta1` scales probabilities when going **from high → medium** energy,
   - `beta2` scales probabilities when going **from high → low** energy.  
   For example, `q_high_pitch_c1 * beta1` is:
   > P(success at medium level | previously high level, same conditions)

3. **Topology & staggering coefficients**  
   - `theta` modifies base probabilities when switching from **line → rectangle**.  
     E.g. `(q_w_high_c1 * theta) * q_high_pitch_c1`  
     ≈ P(LED & buzzer success | rectangle motif, high state).  
   - `gamma` modifies probabilities in **staggered mode** to account for
     staggered vs synchronous execution statistics.

4. **Context-dependent guards**  
   Each transition combines these constants with **guards**:
   - motif: `Rec_motif1` vs `!Rec_motif1`,
   - execution mode: `staggered` vs `!staggered`,
   - hardware state: `hw_state`,
   - environment state: `env_state*`, `sound`, `move`.  

   This means each probability is really a **conditional probability** such as:

   > P(next_state = finish | current_state = high_blink_high_buzz,  
   > color = red, motif = rectangle, staggered = true, hw_state = 0, env idle)

   or

   > P(task_failed | cluster in high state, movement = 1, hw_state = 0, line motif)

5. **Joint probabilities via products**  
   Terms like `q_w_high_c1 * q_high_pitch_c1` implicitly assume **approximate independence** between:
   - LED success,
   - buzzer success,  
   so that we approximate:
   > P(LED OK ∧ buzzer OK | context) ≈ P(LED OK | context) · P(buzzer OK | context)

---

## What has been upgraded compared to earlier versions

Compared to earlier, simpler MDPs / DTMCs, this full model now includes:

- **Two explicit clusters (`c1`, `c2`)** instead of a single abstract BB.
- **Explicit environment module**:
  - MIC / ACC / BOTH sensing,
  - movement and sound flags influencing transitions.
- **Explicit hardware module**:
  - 3-level current feedback controlling fallback transitions.
- **Motif modeling**:
  - line vs rectangle patterns with `theta` coefficient.
- **Staggered vs synchronous execution**:
  - `Operator.staggered` + `[step]` / `[stag1]` labels,
  - plus `turn` logic in `Task_Controller` to coordinate clusters.
- **Energy-aware fallback logic**:
  - systematic use of `beta1`, `beta2` to adjust success/failure at lower levels,
  - prioritization: hardware → environment → default empirical behavior.
- **Simultaneous actuation states**:
  - joint LED + buzzer states (`high_blink_high_buzz`, …) with joint probabilities.

---

## What is still missing / planned future work

This folder **does not yet** contain a fully “finished” model. Planned upgrades:

1. **Parameter calibration**
   - Instantiate all constants (`theta`, `beta1`, `beta2`, `gamma`, `alpha2`, `q_*`) with values derived from **real measurements**:
     - per-color success rates,
     - per-pitch success rates,
     - line vs rectangle experiments,
     - staggered vs simultaneous execution data.

2. **Cluster 2 refinement**
   - Currently, `c2` mostly mirrors `c1`.
   - `alpha2` will be used to model:
     - **increased failure probability** in cluster 2 due to distance from the power source and wiring constraints.

3. **Reward structures & properties**
   - Add reward structures for:
     - energy consumption,
     - number of successful tasks,
     - failure penalties.
   - Add `.props` / `.csl` file with PCTL / CSL properties for verification:
     - probability of task success,
     - expected energy use,
     - comparison between motifs and between staggered vs synchronous.

4. **Tighter link to `usercode.c/h`**
   - Document and implement a clean mapping:
     - from PRISM states → firmware functions / API calls,
     - from hardware/Env feedback → real sensor readings (current sensor, MIC, ACC).

5. **Cleanup & simplification**
   - Review guards like `turn=0` for staggered mode and decide where they are strictly needed.
   - Factor out repeated patterns using PRISM formulae/macros where possible, to improve readability.

---
