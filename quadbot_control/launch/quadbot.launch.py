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

    # Настраиваем ноду для ros2_control
    control_node = Node(
        package="controller_manager",       # Указываем пакет
        executable="ros2_control_node",     # Указываем исполняемый файл
        parameters=[robot_controllers],     # Передаем в него параметры контроллеров
        output="both",                      # Указываем куда выводить логи (both - терминал + ~/.ros/log/YYYY-MM-DD-HH-MM-SS-*.log.)
    )

    # Настраиваем ноду для robot_state_publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",    # Указываем пакет
        executable="robot_state_publisher", # Указываем исполняемый файл
        parameters=[robot_description],     # Передаем в него описание робота
        output="both",                      # Указываем куда выводить логи (both - терминал + ~/.ros/log/YYYY-MM-DD-HH-MM-SS-*.log.)
    )

    # Настраиваем ноду для rviz2
    rviz_node = Node(
        package="rviz2",                                                                                            # Указываем пакет
        executable="rviz2",                                                                                         # Указываем исполняемый файл
        name="rviz2",                                                                                               # Указываем свое имя ноды, иначе будет дано уникальное
        output="log",                                                                                               # Вывод логов только в логи (~/.ros/log/YYYY-MM-DD-HH-MM-SS-*.log.)
        arguments=["-d", PathJoinSubstitution([FindPackageShare("quadbot_control"), "config", "quadbot.rviz"])],    # Аргумент для графического интерфейса, иначе будут параметры по умолчанию
        condition=IfCondition(gui),                                                                                 # Нода стартует только если указанный аргумент "true"
    )

    # Запускает указанный в конфигурации контроллер с помощью исполняемого файла spawner из controller manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",           # Указываем пакет
        executable="spawner",                   # Указываем исполняемый файл
        arguments=["joint_state_broadcaster"],  # Указываем необходимый контроллер в параметрах
    )

    # Запускает указанный в конфигурации контроллер с помощью исполняемого файла spawner из controller manager
    robot_controller_spawner = Node(
        package="controller_manager",                                               # Указываем пакет
        executable="spawner",                                                       # Указываем исполняемый файл
        arguments=["quadbot_base_controller", "--param-file", robot_controllers],   # Указываем необходимый контроллер и его параметры для более точной, относительно joint_state_broadcaster работы
        remappings=[('/cmd_vel', '/cmd_vel_unstamped')],  # ← КЛЮЧЕВОЕ!
    )

    # Указываем задержку для запуска ноды rviz2
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(                        # Действие, которое ожидаем (процесс завершился)
            target_action=joint_state_broadcaster_spawner,  # Объект отслеживания
            on_exit=[rviz_node]),                           # Объект выполнения после действия
    )

    # Указываем задержку для запуска ноды robot_controller_spawner
    delay_controller = RegisterEventHandler(
        event_handler=OnProcessExit(                        # Действие, которое ожидаем (процесс завершился)
            target_action=joint_state_broadcaster_spawner,  # Объект отслеживания
            on_exit=[robot_controller_spawner]),            # Объект выполнения после действия
    )

    # Собирает итоговый файл для запуска и аргументы указанные при вызове лаунча
    return LaunchDescription(
        declared_arguments + [
        control_node, 
        robot_state_pub_node, 
        joint_state_broadcaster_spawner,
        #delay_rviz, 
        delay_controller
    ])
