# Minimum physical-contact adapter

Scope authorized by the task: independent additive package based on PR #1 at
97f47cc. No edits to the synthetic controller, executor, launch or defaults.

Use a ZeroMQ stepping client and a generated ODE scene: fixed static polygonal
bore, dynamic cylinder, three motorized prismatic axes with intervening masses,
and a rigid six-axis force sensor between carriage and peg. This is a Cartesian
contact test rig, not a six-axis robot/IK model. Geometry and engine settings are
JSON inputs. No binary scene is claimed generated without CoppeliaSim.

Use existing TwistStamped, PoseStamped and WrenchStamped topics, robot_base
aligned to world, SI units. Tool origin is peg tip. Rotate sensor wrench and
shift moment to the tip in telemetry; publish the ROS wrench at the base origin
so its frame_id has standard moment-origin semantics. Keep native readings separately; require a deliberate
reaction-sign choice before motion. Physical servo uses a separate negative
lateral gain to follow reaction away from the wall, not the synthetic error sign.

Single owner steps at 20 ms, ODE substeps 2 ms, gravity zero to isolate contact.
ROS uses wall stamps because existing guards/timers use wall time. Simulation
time is separately recorded in telemetry. Reject invalid commands, stale stamps,
nonzero angular commands, foreign frames and time jumps. Bound speed, workspace
and force with a latched stop. Invalid force data suppresses feedback and motion.
RPC loss freezes a stepping simulation; never describe that as a safety-rated stop.

Build stopped empty scene only; refuse an existing prototype. No scene clearing.
Scenario names are interventions, not ground-truth labels. Normal insertion,
misalignment, sidewall contact, blockage and overload need measured pose,
contact handles/points and force evidence. Stagnation plus force is only a jam
candidate. A removable physical stop supplies reproducible blockage.

Deliver offline geometry/frame/guard/transport tests, ROS integration tests with
an injected transport (explicitly not physics), opt-in live-engine probe,
deployment/recording instructions and evidence boundaries. Physical validity
requires calibration, step/friction/compliance sensitivity and live scenario runs.
