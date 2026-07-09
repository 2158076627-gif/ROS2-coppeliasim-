# ros2机械臂装配智能体 v0.1.0

Release date: 2026-07-06

## Highlights

- Reorganized the original robot vision and assembly materials into three product modules:
  - vision recognition
  - trajectory planning
  - force tactile multimodal sensing and impedance servo compliant feedback control
- Added a complete `assembly_trajectory_planner` ROS 2 package.
- Added a complete `compliant_servo_control` ROS 2 package.
- Added the application launch file:

```bash
ros2 launch axis_hole_bringup assembly_agent_demo.launch.py
```

## Included Assets

- ROS 2 source workspace under `src/`
- curated reference prototypes under `reference/`
- module documentation under `modules/`
- architecture and workflow documents under `docs/`
- mock stereo images and point cloud data under `src/demo_data/resource/mock_data/`

## Known Limitations

- The default workflow uses mock image, point cloud, and wrench data.
- Real MoveIt 2 / ros2_control execution is exposed as an integration point, not bundled as a hardware driver.
- Calibration modules are scaffolded and should be filled with hardware-specific procedures before production use.

## Verification

Core Python tests:

```bash
python -m pytest src/assembly_trajectory_planner/test src/compliant_servo_control/test
```

Existing perception tests:

```bash
python src/stereo_hole_detector/test/test_geometry_refiner.py
python src/laser_pointcloud_processor/test/test_circle_fitter.py
python src/axis_hole_fusion/test/test_coordinate_transformer.py
```
