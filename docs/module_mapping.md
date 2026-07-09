# Module Mapping

This document records how the original folder was reorganized into the release application.

## Vision Recognition

Source materials:

- `claude_code/src/stereo_hole_detector`
- `claude_code/src/laser_pointcloud_processor`
- `claude_code/src/axis_hole_fusion`
- `pycharm-code/article_robot_code.py`
- `pycharm-code/github01.py`
- OpenMV and Arduino color-recognition scripts

Release location:

- `src/stereo_hole_detector`
- `src/laser_pointcloud_processor`
- `src/axis_hole_fusion`
- `reference/vision_recognition`

## Trajectory Planning

Source materials:

- `pycharm-code/dummy_sim.py`
- `Github-code/ai_studio_code.py`
- planning fragments in `pycharm-code/UR5_ros6_code.py`

Release location:

- `src/assembly_trajectory_planner`
- `reference/trajectory_planning`

## Force Tactile Multimodal Sensing and Compliant Control

Source materials:

- `pycharm-code/UR5_ros6_code.py`
- `pycharm-code/visual-recognition.py`

Release location:

- `src/compliant_servo_control`
- `reference/compliant_control`

## Integration Entry Point

The application-level launch file is:

```bash
ros2 launch axis_hole_bringup assembly_agent_demo.launch.py
```
