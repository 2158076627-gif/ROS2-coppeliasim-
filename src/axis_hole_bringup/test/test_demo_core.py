import math
import unittest
from assembly_trajectory_planner.planner_core import AssemblyTrajectoryPlanner, PlannerConfig, Pose3D
from compliant_servo_control.admittance_controller import AdmittanceController, WrenchSample

class LoopTests(unittest.TestCase):
    def test_feedback_reaches_contact_then_completes_and_retracts(self):
        from axis_hole_bringup.demo_core import DemoPlant, Executor
        plant = DemoPlant()
        executor = Executor()
        plan = AssemblyTrajectoryPlanner(PlannerConfig(
            observe_distance_m=.03, approach_distance_m=.02,
            contact_distance_m=.002, insertion_depth_m=.004,
            retract_distance_m=.01, coarse_tolerance_m=.001)).plan(Pose3D((.4, 0., .2)))
        controller = AdmittanceController()
        contacted = False
        minimum_z = plant.position[2]
        for _ in range(12000):
            correction = controller.compute(plant.wrench())
            contacted |= correction.contact
            velocity = executor.step(plan, plant.position, correction, .02)
            plant.step(velocity, .02)
            minimum_z = min(minimum_z, plant.position[2])
            if executor.done:
                break
        self.assertTrue(contacted)
        self.assertTrue(executor.done)
        self.assertLess(minimum_z, .198)
        self.assertAlmostEqual(plant.position[2], .21, delta=.0011)
        self.assertEqual(executor.step(plan, plant.position, correction, .02), (0., 0., 0.))

    def test_invalid_wrench_never_commands_motion(self):
        for field in ('fx', 'fy', 'fz', 'tx', 'ty', 'tz'):
            for value in (math.nan, math.inf):
                output = AdmittanceController().compute(WrenchSample(**{field: value}))
                self.assertTrue(output.emergency_stop)
                self.assertEqual((output.vx, output.vy, output.vz), (0., 0., 0.))

    def test_invalid_axis_is_rejected(self):
        with self.assertRaises(ValueError):
            AssemblyTrajectoryPlanner().plan(Pose3D((0., 0., 0.)), (math.nan, 0., 1.))

    def test_overload_latches_executor_stop(self):
        from axis_hole_bringup.demo_core import Executor
        plan = AssemblyTrajectoryPlanner().plan(Pose3D((.4, 0., .2)))
        executor = Executor()
        stop = AdmittanceController().compute(WrenchSample(fz=50.))
        self.assertEqual(executor.step(plan, (.4, 0., .25), stop, .02), (0., 0., 0.))
        clear = AdmittanceController().compute(WrenchSample())
        self.assertEqual(executor.step(plan, (.4, 0., .25), clear, .02), (0., 0., 0.))

    def test_source_and_receive_timeout_frame_and_nan_guards(self):
        from compliant_servo_control.input_guard import InputGuard
        guard = InputGuard(.5)
        self.assertFalse(guard.fresh(10**9))
        self.assertTrue(guard.accept('robot_base', 10**9, 10**9, (1.,)))
        self.assertTrue(guard.fresh(10**9))
        self.assertFalse(guard.fresh(2*10**9))
        for frame, stamp, value in [('tool0', 10**9, 1.), ('robot_base', 1, 1.),
                                     ('robot_base', 2*10**9, 1.), ('robot_base', 10**9, math.nan)]:
            self.assertFalse(guard.accept(frame, stamp, 10**9, (value,)))
            self.assertFalse(guard.fresh(10**9))
        guard.accept('robot_base', 10**9, 10**9)
        guard.received -= .6
        self.assertFalse(guard.fresh(10**9))

    def test_contact_feedback_changes_axial_speed(self):
        c = AdmittanceController()
        free = c.compute(WrenchSample())
        contact = c.compute(WrenchSample(fz=14.))
        self.assertLess(abs(contact.vz), abs(free.vz))
        self.assertLess(contact.vz, 0.)
        lateral = c.compute(WrenchSample(fx=10., ty=.6))
        self.assertLess(lateral.vx, 0.)
        self.assertLess(lateral.wy, 0.)
        self.assertLessEqual(abs(lateral.vx), c.config.max_linear_speed_mps)

    def test_large_finite_force_stops_without_overflow(self):
        correction = AdmittanceController().compute(WrenchSample(fx=1e308))
        self.assertTrue(correction.emergency_stop)
        self.assertEqual((correction.vx, correction.vy, correction.vz), (0., 0., 0.))
