# Paquete de Control para Rosbot (controldesc_pkg)


## 📂 Estructura del Workspace

Copiar esta carpeta dentro de rosbot_ws (practica 2):

rosbot_ws/
├── build/
├── install/
├── log/
└── src/
    ├── rosbot_ros/         <-- Repositorio de Husarion (ya existente)
    └── controldesc_pkg/    <-- ESTE PAQUETE

## 🚀 Puesta en marcha

### 1. Compilación inicial
Este paso es necesario la primera vez que descargas el paquete o si modificas el `CMakeLists.txt` o `package.xml`.

cd ~/rosbot_ws
colcon build --packages-select controldesc_pkg --symlink-install

### 2. Lanzar la Simulación (Terminal 1)
Esto abrirá Gazebo (el mundo 3D) y RViz (visualización de sensores).

cd ~/rosbot_ws
source install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot

### 3. Lanzar el Controlador (Terminal 2)
Esto ejecutará tu lógica de control.

cd ~/rosbot_ws
source install/setup.bash
ros2 run controldesc_pkg robot_controller.py

## 🛠️ Guía de Desarrollo y Solución de Problemas

### Error: "No executable found"
Si al intentar ejecutar un nodo te sale un error de permisos, ejecuta:

chmod +x src/controldesc_pkg/src/robot_controller.py

### Cómo añadir un nuevo script Python (.py)

Si creas un archivo nuevo (ej: `otro_nodo.py`), sigue estos pasos obligatorios:

**1. Dar permisos de ejecución:**

chmod +x src/controldesc_pkg/src/otro_nodo.py

**2. Registrarlo en `CMakeLists.txt`:**
Busca la sección de Python y añade tu archivo:

# ---------- Python ----------
install(
  PROGRAMS
  src/robot_controller.py
  src/otro_nodo.py  # <--- AÑADE ESTA LÍNEA
  DESTINATION lib/${PROJECT_NAME}
)

**3. Recompilar:**
Para que los cambios surtan efecto:

cd ~/rosbot_ws
colcon build --packages-select controldesc_pkg --symlink-install
source install/setup.bash
