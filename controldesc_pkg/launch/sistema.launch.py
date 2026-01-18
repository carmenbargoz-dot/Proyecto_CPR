import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# --- FUNCIÓN 1: SELECCIONAR GAZEBO ---
def lanzar_gazebo_segun_escenario(context, *args, **kwargs):
    nombre_escenario = LaunchConfiguration('escenario').perform(context)
    pkg_rosbot_gazebo = get_package_share_directory('rosbot_gazebo')
    pkg_husarion_worlds = get_package_share_directory('husarion_gz_worlds')

    if nombre_escenario == 'husarion_world':
        print(f"--- GAZEBO: Cargando mundo standard ({nombre_escenario}) ---")
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rosbot_gazebo, 'launch', 'simulation.launch.py')
                ),
                launch_arguments={'robot_model': 'rosbot'}.items()
            )
        ]
    else:
        print(f"--- GAZEBO: Cargando mundo custom ({nombre_escenario}) ---")
        ruta_mundo_sdf = os.path.join(pkg_husarion_worlds, 'worlds', nombre_escenario + '.sdf')
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_rosbot_gazebo, 'launch', 'office.launch.py')
                ),
                launch_arguments={
                    'robot_model': 'rosbot',
                    'gz_world': ruta_mundo_sdf
                }.items()
            )
        ]

# --- FUNCIÓN 2: SELECCIONAR PLANIFICADOR ---
def lanzar_planificador(context, *args, **kwargs):
    # Leemos el argumento 'planificador'
    nombre_plan = LaunchConfiguration('planificador').perform(context)
    
    # Mapeamos la clave al nombre del archivo real
    archivo_ejecutable = ''
    
    if nombre_plan == 'Astar':
        archivo_ejecutable = 'A_star_mejor.py'
    elif nombre_plan == 'Theta':
        archivo_ejecutable = 'Theta_star.py'
    elif nombre_plan == 'Theta2':
        archivo_ejecutable = 'Theta_star2.py'
    else:
        # Si escriben algo raro, usamos Astar por seguridad también
        print(f"--- AVISO: Planificador '{nombre_plan}' no reconocido. Usando Astar por defecto ---")
        archivo_ejecutable = 'A_star_mejor.py'

    print(f"--- PLANIFICADOR: Lanzando {archivo_ejecutable} ---")

    return [
        Node(
            package='controldesc_pkg',
            executable=archivo_ejecutable,
            name='planner_node',
            output='screen'
        )
    ]

def generate_launch_description():
    
    # --- 1. RUTAS ---
    pkg_controldesc = get_package_share_directory('controldesc_pkg')
    user_home = os.environ.get('HOME')
    path_to_maps_dir = os.path.join(
        user_home, 'rosbot_ws', 'controldesc_pkg', 'src', 'A_star_controller', 'maps'
    )

    # --- 2. ARGUMENTOS ---
    escenario_arg = DeclareLaunchArgument(
        'escenario',
        default_value='husarion_world', 
        description='Nombre del mundo/mapa (ej: husarion_world, warehouse)'
    )
    
    # --- CAMBIO AQUÍ: Por defecto ahora es Astar ---
    planificador_arg = DeclareLaunchArgument(
        'planificador',
        default_value='Astar',  # <--- CAMBIADO
        description='Selecciona algoritmo: Astar, Theta, o Theta2'
    )

    escenario_config = LaunchConfiguration('escenario')

    # --- 3. CONFIGURACIÓN MAPA ---
    map_path = PathJoinSubstitution([
        path_to_maps_dir, [escenario_config, '.yaml']
    ])

    # --- 4. NODOS ESTÁTICOS ---
    
    gazebo_decision = OpaqueFunction(function=lanzar_gazebo_segun_escenario)
    planner_decision = OpaqueFunction(function=lanzar_planificador)

    map_publisher_node = Node(
        package='controldesc_pkg',
        executable='map_publisher.py',
        name='map_publisher',
        output='screen',
        parameters=[{'map_yaml': map_path}]
    )

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_controldesc, 'launch', 'amcl_launch.py')
        )
    )

    controller_node = Node(
        package='controldesc_pkg',
        executable='robot_controller.py',
        name='robot_controller',
        output='screen'
    )

    init_movement = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/cmd_vel', 'geometry_msgs/msg/Twist',
                    '{linear: {x: 0.0}, angular: {z: 0.1}}'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        escenario_arg,
        planificador_arg,
        gazebo_decision,
        planner_decision,
        map_publisher_node,
        amcl_launch,
        controller_node,
        init_movement
    ])
