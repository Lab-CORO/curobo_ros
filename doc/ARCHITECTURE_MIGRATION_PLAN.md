# Architecture Migration Plan

> **Date**: November 13, 2025
> **Purpose**: Migrate from `curobo_gen_traj` node to unified `unified_planner` architecture and update robot setup to use `curobo_robot_setup` RViz plugin

---

## Overview

### Changes to Implement

1. **Node Architecture**
   - **OLD**: `curobo_gen_traj` node
   - **NEW**: `unified_planner` node

2. **Robot Configuration Tool**
   - **OLD**: NVIDIA Isaac Sim + Lula Robot Description Editor
   - **NEW**: `curobo_robot_setup` RViz2 plugin (https://github.com/Lab-CORO/curobo_robot_setup)

---

## Impact Analysis

### Files Affected
- **209 occurrences** of `curobo_gen_traj` across 15 markdown files
- Tutorial files: All tutorials reference the old node name
- Launch files: References to `gen_traj.launch.py`
- Service examples: All service calls use `/unified_planner/...` namespace

### Service/Topic Name Changes

| Old Namespace | New Namespace |
|---------------|---------------|
| `/unified_planner/generate_trajectory` | `/unified_planner/generate_trajectory` |
| `/unified_planner/add_object` | `/unified_planner/add_object` |
| `/unified_planner/remove_object` | `/unified_planner/remove_object` |
| `/unified_planner/get_obstacles` | `/unified_planner/get_obstacles` |
| `/unified_planner/collision_spheres` | `/unified_planner/collision_spheres` |
| `/unified_planner/*` | `/unified_planner/*` |

---

## Migration Strategy

### Phase 1: Robot Setup Tutorial (Priority: HIGH)

**File**: `doc/tutorials/2_adding_your_robot.md`

**Current State**:
- Method 1: Isaac Sim (8 steps, ~1 hour, requires 30GB Isaac Sim)
- Method 2: Manual (6 steps, 2-4 hours)

**New State**:
- **Method 1 (Recommended)**: `curobo_robot_setup` RViz plugin (simple, integrated)
- **Method 2**: Manual (fallback)

**curobo_robot_setup Tool**:
- RViz2 plugin (no extra software needed)
- 3 tabs: URDF loading, Collision spheres, Configuration export
- Interactive GUI for sphere editing
- Direct YAML export for cuRobo
- License: GPL-3.0
- Maintainer: Will-44

**New Tutorial Structure**:
```
## Method 1: Using curobo_robot_setup RViz Plugin (Recommended)

### Prerequisites
- ROS 2 (Humble+)
- RViz2
- Your robot URDF

### Step 1: Install curobo_robot_setup
cd ~/ros2_ws/src
git clone https://github.com/Lab-CORO/curobo_robot_setup.git
cd ~/ros2_ws
colcon build --packages-select curobo_robot_setup
source install/setup.bash

### Step 2: Launch RViz with Plugin
rviz2

### Step 3: Add CuRobo Setup Panel
Panels → Add New Panel → CuRoboSetupPanel

### Step 4: Load URDF (Tab 1)
- Click "Load URDF"
- Select your robot URDF file
- Robot appears in RViz

### Step 5: Define Collision Spheres (Tab 2)
- Select link from dropdown
- Click "Add Sphere"
- Adjust position [x, y, z] and radius
- Spheres appear as interactive markers in RViz
- Repeat for all links

### Step 6: Configure Parameters (Tab 3)
- Set base_link and ee_link
- Configure joint limits
- Set retract position
- Adjust weights

### Step 7: Export Configuration
- Click "Export YAML"
- Save as my_robot.yml
- Configuration ready for curobo_ros!
```

### Phase 2: Quick Start and README (Priority: HIGH)

**File**: `README.md`

**Changes**:
```bash
# OLD
ros2 launch curobo_ros unified_planner.launch.py
ros2 service call /unified_planner/generate_trajectory ...

# NEW
ros2 launch curobo_ros unified_planner.launch.py
ros2 service call /unified_planner/generate_trajectory ...
```

### Phase 3: Getting Started Guide (Priority: HIGH)

**File**: `doc/getting_started.md`

**Changes**:
- Update launch command
- Update service examples
- Update parameter names if changed

### Phase 4: All Tutorials (Priority: MEDIUM)

**Files to update** (15 files):
1. `doc/tutorials/1_first_trajectory.md` - Update all examples
2. `doc/tutorials/2_adding_your_robot.md` - Replace Isaac Sim section
3. `doc/tutorials/4_dynamic_strategy_switching.md`
4. `doc/tutorials/adding_collision_objects.md` - Many service calls
5. `doc/tutorials/doosan_example.md`
6. `doc/tutorials/dynamic_strategy_switching.md`
7. `doc/tutorials/pointcloud_obstacle_detection.md`
8. `doc/concepts/introduction.md`
9. `doc/concepts/parameters.md`
10. `doc/concepts/ros_interfaces.md`
11. `doc/concepts/warmup_async.md`
12. `srv/README.md`
13. `curobo_ros/robot/README.md`

**Systematic replacement**:
- `/unified_planner/` → `/unified_planner/`
- `unified_planner node` → `unified_planner node`
- Parameter names: `/unified_planner/robot_config_file` → `/unified_planner/robot_config_file`

### Phase 5: Concept Documentation (Priority: LOW)

Update architecture diagrams and explanations to reflect unified planner.

---

## Implementation Checklist

### Robot Setup Tutorial
- [ ] Remove Isaac Sim Method 1 (8 steps)
- [ ] Create new Method 1 with curobo_robot_setup (7 steps)
- [ ] Add screenshots/diagrams of RViz plugin
- [ ] Update installation prerequisites
- [ ] Add troubleshooting section for RViz plugin
- [ ] Update comparison table
- [ ] Test workflow and verify accuracy

### Node Name Updates
- [ ] README.md Quick Start section
- [ ] README.md documentation links
- [ ] getting_started.md launch commands
- [ ] getting_started.md service examples
- [ ] Tutorial 1: First Trajectory
- [ ] Tutorial 2: Adding Your Robot
- [ ] Tutorial 4: Dynamic Strategy Switching
- [ ] Tutorial: Adding Collision Objects (many occurrences!)
- [ ] Tutorial: Doosan Example
- [ ] Tutorial: Point Cloud Obstacle Detection
- [ ] Concept: Introduction
- [ ] Concept: Parameters
- [ ] Concept: ROS Interfaces
- [ ] Concept: Warmup Async
- [ ] srv/README.md

### Verification
- [ ] All service names updated
- [ ] All topic names updated
- [ ] All parameter names updated
- [ ] All launch file references updated
- [ ] Code examples tested (if possible)
- [ ] Cross-references between docs verified
- [ ] No broken links

---

## Testing Notes

After migration, verify:
1. Launch command works: `ros2 launch curobo_ros unified_planner.launch.py`
2. Services are available: `ros2 service list | grep unified_planner`
3. Topics are publishing: `ros2 topic list | grep unified_planner`
4. RViz plugin loads: `rviz2` → Add CuRoboSetupPanel

---

## Rollback Plan

If issues arise:
1. Git branch with old documentation: `main` (before migration)
2. Old node may still exist: check if `curobo_gen_traj` is deprecated or removed
3. Keep migration as separate commit for easy revert

---

## Benefits of New Architecture

### Unified Planner Node
- Cleaner architecture with planner abstraction
- Support for multiple planner types (Classic, MPC, Batch, Constrained)
- Runtime planner switching
- Better code organization

### curobo_robot_setup Plugin
- **No Isaac Sim required**: Saves ~30 GB disk space, easier installation
- **Integrated workflow**: Everything in RViz2, familiar to ROS users
- **Faster iteration**: Real-time visual feedback, quick adjustments
- **Lower barrier to entry**: No NVIDIA software stack needed
- **Better for CI/CD**: Can run headless, easier to script
- **Open source**: GPL-3.0, community can contribute

---

## Timeline Estimate

- **Phase 1** (Robot Setup Tutorial): 2-3 hours
- **Phase 2** (README): 30 minutes
- **Phase 3** (Getting Started): 1 hour
- **Phase 4** (All Tutorials): 3-4 hours
- **Phase 5** (Concepts): 1 hour
- **Testing & Verification**: 2 hours

**Total**: ~10-12 hours of documentation work
