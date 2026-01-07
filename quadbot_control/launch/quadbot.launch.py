# Библиотеки
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Функция инициирующая запуск
def generate_launch_description():
    # Аргументы, объявляемые при запуске
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true", description="Use mock hardware"),
    ]

    # Переменные для аргументов
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Находим файл с описанием рообота и парсим его с помощью xacro
        # Command - выполняет внешнюю shell команду и возвращает вывод как строку
        # PathJoinSubstitution - безопасно склеивает пути, учитывая особенности разных операционных систем
        # FindExecutable - находит полный путь к исполняемомоу файлу в системе (аргументом нужно передать имя)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", PathJoinSubstitution([FindPackageShare("quadbot_control"), "urdf", "quadbot.urdf.xacro"]),
        " ", "use_mock_hardware:=", use_mock_hardware,
    ])

    # Словарь с описанием робота для robot_state_publisher (он ожидает параметр с именем "robot_description" (точно таким), 
    # содержащий строку с URDF)
    robot_description = {"robot_description": robot_description_content}

    # Получаем параметры контроллеров в переменную
    robot_controllers = PathJoinSubstitution([FindPackageShare("quadbot_control"), "config", "quadbot_controllers.yaml"])

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
