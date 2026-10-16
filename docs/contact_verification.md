# Contact adapter verification — 2026-10-16

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
NOT a physics test and did not run locally. The existing GitHub workflow will
exercise it on Jazzy when this PR is submitted. Check that run for its outcome;
PR #1 CI does not establish this branch's behavior.

Not executed: scene generation/save/load, real ZeroMQ round trips, ODE engine,
force sign calibration, friction/ERP/CFM sensitivity, all physical scenarios,
physical ROS feedback loop, rosbag recording/replay, robot IK or real hardware.
The opt-in live test is only free-motion/sensor smoke coverage; it is not the
five-case physical validation. Follow docs/coppeliasim_contact.md for those.

CI run38906333094 built all four packages and passed package tests, then aborted
on the new ROS bridge pose publication with integer-valued injected zero fields.
The bridge now casts numeric pose/wrench fields to Python float at the ROS
serialization boundary; the injected integer fixture is kept as regression input.
The corrected commit requires a new Jazzy run; no physical result is implied.
