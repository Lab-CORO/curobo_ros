# Testing

`curobo_ros` ships 7 integration suites (`config/config_test/*.yaml` → generated `test/test_*.py`) plus the standard `ament_copyright` / `ament_flake8` / `ament_pep257` linters. This page covers how to run them and how the generator that produces them works.

## Running the tests

### The normal way — `colcon test`

```bash
colcon test --packages-select curobo_ros
colcon test-result --all --verbose
```

This runs all 10 tests (7 launch suites + 3 linters) and reports a JUnit summary. No special flags are needed.

A full run takes several minutes: each suite launches the real `unified_planner` node, waits for it to build its cuRobo solvers, then exercises services and topics against it.

### One suite at a time — `launch_test`

Useful while iterating on a single YAML, since you see live output instead of waiting for the whole package:

```bash
launch_test test/test_test_kinematics.py
```

### All suites, outside colcon — `run_all_tests.sh`

```bash
test/run_all_tests.sh
```

Generated alongside the suites; loops `launch_test` over all 7 with per-suite timeouts. Same tests as `colcon test`, without the linters and without a JUnit report.

## The generator — `ros2_test_compose`

Nothing under `test/` is hand-written except `test_flake8.py` / `test_copyright.py` / `test_pep257.py`. Everything else — the 7 `test_test_*.py` suites and `run_all_tests.sh` — is generated from `config/config_test/*.yaml` and carries a `DO NOT EDIT` banner. **Never edit a generated file by hand**: the fix belongs in the YAML, or in the generator itself if the YAML can't express it. A hand-patched generated file silently regresses the next time anyone reruns the generator.

Regenerate after touching any `config/config_test/*.yaml`:

```bash
ros2 run ros2_test_compose test_generator --ros-args -p package_name:=curobo_ros
```

To check the generator itself hasn't drifted (e.g. after changing `ros2_test_compose`), regenerate into a scratch copy and diff instead of overwriting in place — see the "Checking the generator hasn't drifted" section of `ros2_test_compose/README.md`.

### Key YAML settings (`environment:` block)

- **`ready_service` / `ready_wait`** — how the suite knows the node is up. `setUpClass` polls `get_service_names_and_types()` until the named service is *advertised* (it is never called — observing the graph can't hang even if a service callback is broken). `unified_planner` builds its solvers synchronously in `__init__` (25–35 s), so all 7 curobo_ros YAMLs set:
  ```yaml
  ready_service: "/unified_planner/generate_trajectory"
  ready_wait: 180.0
  startup_delay: 0.0
  ```
  There is no readiness *service* to call: the node exposes its state as the `node_is_available` parameter, and `ready_service` deliberately watches the graph instead.

- **`allowable_exit_codes`** *(optional, suite-wide, not per-test)* — exit codes accepted besides `0`. None of the curobo_ros YAMLs set it: all launched processes (planner + `robot_state_publisher` + `static_transform_publisher` + `joint_state_publisher`) exit `0` cleanly. Only add it once a suite is legitimately known to exit some other way (a GUI killed on shutdown, a node that doesn't catch SIGINT).

- **`wait_for_timeout`** *(default 10 s)* — per-test timeout for topic/service checks; a per-test `timeout:` key overrides it. Actions use a separate, larger `action_timeout` (default 60 s) since they run a whole trajectory.

- **Budget a solver rebuild generously.** `set_collision_cache`, `update_motion_gen_config`, the first `add_object` and `attach_object` all rebuild and re-warm the solvers *synchronously*. Measured on an idle Jetson Orin that is ~25 s — but after ~30 minutes of sustained GPU load the SoC throttles and the same rebuild takes ~37 s. Every such call therefore uses `timeout: 90.0`. A budget sitting just above the idle cost does not detect a hang, it just turns the whole suite red on a warm or shared machine.

Full reference for every key: `ros2_test_compose/README.md`.

## Known flakes

Two suites fail intermittently for reasons unrelated to what they assert. Both reproduce on an unchanged tree, so a single red run is not a regression signal — rerun the suite before investigating.

`test_test_object.py::test_05_attach_object` occasionally times out (`Service call to '/unified_planner/attach_object' timed out`). Root cause: MorphIt's stochastic sphere-fit retries (`morphit_sphere_fit: attempt N returned too few spheres`), which can take longer than the test's timeout on a slow attempt.

`test_test_trajectory.py` occasionally fails on `test_exit_codes` alone: every service assertion passes, then the planner process aborts (`SIGABRT`, exit `-6`) a few seconds after the shutdown SIGINT, during CUDA/solver teardown. Recognise it by the log — all numbered tests green, and `process has died [... exit code -6]` *after* `sending signal 'SIGINT'`. Nothing is wrong with the trajectories it just planned.

## Next steps

- [Troubleshooting](troubleshooting.md)
- [Tutorial 1: Your First Trajectory](../tutorials/01-first-trajectory.md)
