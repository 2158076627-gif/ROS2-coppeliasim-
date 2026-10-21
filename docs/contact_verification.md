# Contact adapter verification — historical baseline 2026-10-16

For the 2026-10-21 five-case acceptance increment and reproducible deployment,
see [contact_acceptance.md](contact_acceptance.md). Results below belong to the
original PR #2 baseline; they do not validate the later increment or engine physics.

Base: PR #1, 97f47cc38a3485669805dcfdd232642bfedb11d0.
Environment: cloud Linux, Python3.12, no /opt/ros, rclpy or CoppeliaSim executable.
Installed official ZeroMQ Python client2.0.4 for import/API inspection only;
no running engine was available.

Executed:
- `python -m pytest -q -rs`: 33 passed, 3 skipped, 4 packaging subtests passed.
  Skips: existing ROS loop module, new ROS bridge module, opt-in live engine test.
- `python -m colcon list`: four ros.ament_python packages.
- `python -m colcon build --symlink-install`: four packages finished.
- `python -m colcon test --return-code-on-test-failure`: four packages finished.
- `python -m colcon test-result --verbose`: 10 tests, 0 errors/failures/skips.
  These ten are existing package-local tests; new tests run via repository pytest.
- `python -m compileall -q src tests` and `git diff --check`: exit0.
- Diff of all original three packages against97f47cc: empty.

The recording API double confirms builder calls/parameters, sensor hierarchy,
step reset rejection and contact-record preservation. It is NOT CoppeliaSim.
`test_contact_ros.py` uses real ROS middleware with an injected transport; it is
NOT a physics test and did not run locally. The GitHub workflow has now exercised both ROS modules on Jazzy successfully;
see the commit-specific evidence below. PR #1 CI alone does not establish this branch's behavior.

Not executed: scene generation/save/load, real ZeroMQ round trips, ODE engine,
force sign calibration, friction/ERP/CFM sensitivity, all physical scenarios,
physical ROS feedback loop, rosbag recording/replay, robot IK or real hardware.
The opt-in live test is only free-motion/sensor smoke coverage; it is not the
five-case physical validation. Follow docs/coppeliasim_contact.md for those.

CI run38906333094 built all four packages and passed package tests, then aborted
on the new ROS bridge pose publication with integer-valued injected zero fields.
The bridge now casts numeric pose/wrench fields to Python float at the ROS
serialization boundary; the injected integer fixture is kept as regression input.
The corrected head `c5aa35c0d6713c8680c393591f212528c528dc59` subsequently passed
[Jazzy run 38906535641](https://github.com/2158076627-gif/ROS2-coppeliasim-/actions/runs/38906535641)
on 2026-10-16 UTC. Job `116991391761` checked out PR merge preview `450ae11`
(head c5aa35c into base 97f47cc). Its log reports:

```text
colcon build: 4 packages finished
colcon test-result: 10 tests, 0 errors, 0 failures, 0 skipped
python3 -m pytest -q -rs: 38 passed, 1 skipped in 37.51s
SKIPPED tests/test_contact_live.py:6: CoppeliaSim live engine not requested
```

The old four process tests and the new injected-backend ROS bridge test ran.
The ten package-local tests are included in repository pytest, not ten additional
independent tests. This resolves the serialization/ROS-interface CI gap only;
no physical result is implied. Documentation-only follow-up commits need their
own CI result; this historical run is tied to c5aa35c, not any later SHA.

## Merge and physical acceptance gates

PR #1 supplies the numerical baseline (97f47cc, run 38857876199: 17 passed).
PR #2 originally targets `fix/minimal-assembly-loop`. Preserve PR #1 ancestry
with a merge commit into main, then retarget PR #2 to main and inspect the new
diff. Do not merge PR #2 into PR #1 first, force-push, or squash/rebase away the
shared baseline. The original three packages and both launch entries remain
independent. CI green and source review do not mark physics accepted.

Keep PR #2 experimental and pending engine acceptance. Before integration as a
physically validated prototype, attach evidence for all of the following:

- Engine version, git SHA, JSON/YAML settings; actual build/save/reload of .ttt,
  ODE/time/material readback, and real ZeroMQ stepping without competing clients.
- Free-motion/sensor smoke test (`COPPELIA_LIVE=1`); low-force axial/lateral
  sign calibration with raw logs and moment-origin checks. Smoke alone is insufficient.
- Dynamic-chain stability and no wall penetration; normal/offset/sidewall/jam/
  overload trials with geometry/contact identities, force, actual depth, stop
  latch and stop distance. Label unmet cases invalid/uncertain, not successes;
  the blocker jam case is not proof of frictional wedging.
- ROS physical feedback loop separately from the direct probe; open/closed-loop
  comparisons, friction/ERP/CFM/time-step sensitivity, rosbag/JSONL recording and
  replay with synchronized metadata. No invented success rate or precision.

Six-axis IK, joint/controller integration, real sensors, hardware protection and
real-robot transfer remain separate future work even after the 3P tests pass.
