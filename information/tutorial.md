Ниже полный проект **четырехколесного робота** с **Python packaging** (`setup.py`). Python-пакеты ROS 2 собираются через `colcon build` точно так же, но используют `setup.py` вместо `CMakeLists.txt`.

## Структура пакета

```text
ros2_ws/
  src/
    quadbot_control/
      package.xml
      setup.py
      setup.cfg
      quadbot_control/
        __init__.py
        launch/
          quadbot.launch.py
      resource/
        quadbot_control
      urdf/
        quadbot.urdf.xacro
      config/
        quadbot_controllers.yaml
        quadbot.rviz
```

## package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>quadbot_control</name>
  <version>0.0.1</version>
  <description>4-wheel quadbot with ros2_control (Python package)</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>launch</depend>
  <depend>launch_ros</depend>
  <depend>xacro</depend>
  <depend>robot_state_publisher</depend>
  <depend>ros2_control</depend>
  <depend>ros2_controllers</depend>
  <depend>controller_manager</depend>
  <depend>rviz2</depend>
  <depend>diff_drive_controller</depend>
  <depend>joint_state_broadcaster</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'quadbot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('quadbot_control/launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='4-wheel quadbot with ros2_control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

## setup.cfg

```ini
[develop]
script-dir=$base/lib/quadbot_control
[install]
install-scripts=$base/lib/quadbot_control
```

## quadbot_control/__init__.py

```python
from pathlib import Path
__version__ = "0.0.1"
```

## resource/quadbot_control

Пустой файл (маркер пакета):
```bash
touch resource/quadbot_control
```

## quadbot_control/launch/quadbot.launch.py

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true", description="Use mock hardware"),
    ]

    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", PathJoinSubstitution([FindPackageShare("quadbot_control"), "urdf", "quadbot.urdf.xacro"]),
        " ", "use_mock_hardware:=", use_mock_hardware,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("quadbot_control"), "config", "quadbot_controllers.yaml"
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution([FindPackageShare("quadbot_control"), "config", "quadbot.rviz"])],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["quadbot_base_controller", "--param-file", robot_controllers],
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node]),
    )
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=joint_state_broadcaster_spawner, on_exit=[robot_controller_spawner]),
    )

    return LaunchDescription(declared_arguments + [
        control_node, robot_state_pub_node, joint_state_broadcaster_spawner,
        delay_rviz, delay_controller
    ])
```

## urdf/quadbot.urdf.xacro

```xml
<?xml version="1.0"?>
<robot name="quadbot" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="use_mock_hardware" default="true"/>

  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.4 0.1"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
    <collision><geometry><box size="0.5 0.4 0.1"/></geometry></collision>
    <inertial><mass value="10"/><inertia ixx="1" iyy="1" izz="1"/></inertial>
  </link>

  <link name="front_left_wheel">
    <visual><geometry><cylinder radius="0.05" length="0.03"/></geometry></visual>
  </link>
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="front_left_wheel"/>
    <origin xyz="0.2 0.15 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="front_right_wheel">
    <visual><geometry><cylinder radius="0.05" length="0.03"/></geometry></visual>
  </link>
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="front_right_wheel"/>
    <origin xyz="0.2 -0.15 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="rear_left_wheel">
    <visual><geometry><cylinder radius="0.05" length="0.03"/></geometry></visual>
  </link>
  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="rear_left_wheel"/>
    <origin xyz="-0.2 0.15 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="rear_right_wheel">
    <visual><geometry><cylinder radius="0.05" length="0.03"/></geometry></visual>
  </link>
  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="rear_right_wheel"/>
    <origin xyz="-0.2 -0.15 -0.05" rpy="-1.5708 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <ros2_control name="QuadBotSystem" type="system">
    <hardware>
      <plugin>ros2_control_demo_hardware/QuadBotSystemHardware</plugin>
      <param name="use_mock_hardware">${use_mock_hardware}</param>
    </hardware>
    <joint name="front_left_wheel_joint">
      <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/>
    </joint>
    <joint name="front_right_wheel_joint">
      <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/>
    </joint>
    <joint name="rear_left_wheel_joint">
      <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/>
    </joint>
    <joint name="rear_right_wheel_joint">
      <command_interface name="velocity"/><state_interface name="position"/><state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
```

## config/quadbot_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    quadbot_base_controller:
      type: diff_drive_controller/DiffDriveController

quadbot_base_controller:
  ros__parameters:
    publish_rate: 50.0
    base_frame_id: base_link
    left_wheel_names: ["front_left_wheel_joint", "rear_left_wheel_joint"]
    right_wheel_names: ["front_right_wheel_joint", "rear_right_wheel_joint"]
    wheel_separation: 0.3
    wheel_radius: 0.05
    use_stamped_vel: false
    enable_odom_tf: true
    odom_frame_id: odom
```

## 🚀 Сборка и запуск

```bash
cd ~/ros2_ws
colcon build --packages-select quadbot_control
source install/setup.bash
ros2 launch quadbot_control quadbot.launch.py gui:=true use_mock_hardware:=true
```

## 🎮 Управление роботом

```bash
# Вперёд
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"

# Поворот
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Teleop клавиатура (новый терминал)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Ключевые отличия Python packaging:**
- `buildtool_depend` → `ament_python`
- `setup.py` + `setup.cfg` вместо `CMakeLists.txt`
- Файлы запуска в `quadbot_control/launch/` (внутренний пакет)
- `data_files` в `setup.py` регистрирует все ресурсы (URDF, config, launch)
- `resource/quadbot_control` — обязательный маркер пакета

Проект полностью готов к сборке через `colcon build`!