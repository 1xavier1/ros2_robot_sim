# Cattle Barn World Implementation Plan

> **For AI agents:** implement inline in small tested edits.

**Goal:** Add a cattle-barn Gazebo world with an outdoor default spawn point and make world switching explicit from launch and shell scripts.

**Architecture:** Keep worlds as SDF files under `worlds/`. `robot_simulation.launch.py` exposes a `world` launch argument and defaults to `cattle_barn.world`. Shell starters resolve short world names to absolute paths and pass them into launch.

**Technical stack:** Gazebo SDF 1.6, ROS 2 launch, Bash start scripts, pytest text-contract tests.

---

### Task 1: Lock World Switch Contracts

**Files:**
- Modify: `src/robot_description/test/test_wheel_encoder_integration.py`

- [x] Add assertions for `cattle_barn.world`, `DeclareLaunchArgument('world')`, default outdoor spawn, and script world options.

### Task 2: Add Cattle Barn SDF

**Files:**
- Create: `worlds/cattle_barn.world`

- [x] Model outdoor yard, 4.0 m feed alley, 5.0 m turn corridor, 3.0 m service alleys, pens, fences, feed piles, and low obstacles.

### Task 3: Wire Launch and Scripts

**Files:**
- Modify: `launch/robot_simulation.launch.py`
- Modify: `start.sh`
- Modify: `scripts/start_full_stack.sh`

- [x] Default simulation to `cattle_barn.world`.
- [x] Allow absolute paths or short names such as `cattle_barn` / `corridor_tunnel`.
- [x] Document the active world in startup logs.

### Task 4: Verify

- [x] Run focused pytest contracts.
- [x] Compile launch file.
- [x] Report exact switching commands.
