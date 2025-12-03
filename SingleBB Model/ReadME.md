# PRISM MDP Model – Single Blinky Block (BB)

This repository contains a PRISM Markov Decision Process (MDP) model of a **single Blinky Block (BB)**.  
The model serves as a **baseline abstraction** that can be later extended to represent **clusters** of BBs arranged in different motifs (e.g., line, rectangle) with or without synchronization between modules.

The goal of this version is to:
- Capture the **local self-adaptive behaviour** of one BB.
- Encode **probabilistic success/failure** of actuation under hardware and environmental constraints.
- Provide a **parameterised and easily tunable template** that can be scaled up to multi-BB clusters.

---

## 1. Model Overview

The model is written in PRISM’s MDP syntax and represents a single BB that can:

- Execute **tasks** at three intensity levels: `high`, `med`, `low`.
- Use different **actuators**:
  - **Pitch** (sound intensity),
  - **LEDs** with three colours (**red**, **blue**, **white**),
  - **Simultaneous LED + pitch** (concurrent actuation).
- Perform **sensing tasks** via:
  - Microphone (`sound`),
  - Motion (`move`),
  - Both (combined sensing).

The BB eventually reaches either:
- A **successful completion** state: `finish`, or  
- A **failure** state: `task_failed`.

Both are modeled as absorbing states, but the BB can be reset and start a new task via a controller.

---

## 2. Modules and State Space

The model is decomposed into several PRISM modules:

### 2.1 `BB1` – Single Blinky Block

`BB1` is the core module and encodes the **behavioural state machine** of one BB:

- **Task selection from `start_task`:**
  - Choose colour: `choose_Red`, `choose_Blue`, `choose_white`.
  - Choose pitch level: `high_pitch`, `med_pitch`, `low_pitch`.
  - Choose sensing task: `listen_on_moving_on`, `listen_on_moving_off`, `listen_off_moving_on`.
- **Actuation states:**
  - LEDs only: `high_Led`, `med_Led`, `low_Led`.
  - Pitch only: `high_pitch`, `med_pitch`, `low_pitch`.
  - Concurrent LED + pitch:
    - `high_blink_high_buzz`,
    - `high_blink_med_buzz`,
    - `med_blink_high_buzz`,
    - `med_blink_med_buzz`.
- **Terminal states:**
  - `finish`,
  - `task_failed`.

The transitions from these states are **probabilistic**, and their distributions depend on:
- Actuation level (high/med/low),
- Chosen colour,
- Hardware state (`hw_state`),
- Environment state (`env_state1`, `sound`, `move`).

### 2.2 `Hardware` – Power / Current Feedback

The `Hardware` module abstracts the **current sensor / hardware feedback** with three levels:

- `hw_state = 0`: normal,
- `hw_state = 1`: medium load,
- `hw_state = 2`: high load.

Depending on `hw_state`, high-consumption states are **forced to downgrade** (e.g. `high_pitch → med_pitch → low_pitch`), or are more likely to fail.

### 2.3 `Environment` – Sensing Abstraction

The `Environment` module represents simplified **environmental conditions**:

- Boolean flags:
  - `sound` (MIC activity),
  - `move` (motion).
- Encoded environment state: `env_state1 ∈ {0,1,2,3}`:
  - `0`: idle,
  - `1`: MIC active,
  - `2`: ACC active,
  - `3`: both active.

Environment transitions influence the BB’s state by:
- Triggering **sensing tasks**, and
- Changing success/failure probabilities for actuation (e.g. movement increasing failure risk at higher intensities).

### 2.4 `Task_Controller` – Task Counter and  modules Activation

The `Feature_Controller` module:

- Maintains a **task counter** (`task_counter ∈ {0,1,2}`),
- Controls whether the BB is **active** (`active1` flag),
- Handles **reset** behaviour after `finish` or `task_failed` via the `[Task_increment]` action.

This abstracts a simple managing layer that can activate/deactivate the BB and repeat tasks.

---

## 3. Parameters, Probabilities, and Abstractions

The model is designed to be **data-driven and easy to tune**. Most of the behaviour is controlled through **user-defined constants** at the top of the file:

### 3.1 Base Probabilities per Colour and Pitch

For example:

```prism
const double q_w_high_c1; // base success prob: white LED at high level
const double q_b_high_c1; // base success prob: blue LED at high level
const double q_r_high_c1; // base success prob: red LED at high level
const double q_high_pitch_c1; // base success prob: high pitch

const double beta1;//=1.022;// success improvement factor when state is MED W.R.T High
const double beta2;//=1.05;// success improvement factor when state is low W.R.T High
