import matplotlib.pyplot as plt
import pandas as pd
import sys

# 1. Cargar datos
try:
    df_orig = pd.read_csv('trayectoria_original.txt')   # Puntos A* (Originales)
    df_smooth = pd.read_csv('trayectoria_suavizada.txt') # Curva Spline (Suavizada)
    df_real = pd.read_csv('trayectoria_real.txt')       # Robot y Descentrado (Real)
except FileNotFoundError:
    print("Error: No se encuentran los archivos .txt.")
    print("Por favor, ejecuta de nuevo el robot para que genere los datos.")
    sys.exit(1)

# 2. Configurar gráfico
plt.figure(figsize=(10, 8))

# --- GRAFICADO (USANDO .values PARA UBUNTU 22) ---

# A) Puntos Originales (A*) -> Puntos Verdes Grandes
plt.plot(df_orig['x'].values, df_orig['y'].values, 
         'go', label='Puntos Originales', markersize=6)

# B) Trayectoria Suavizada -> Línea Negra Punteada
plt.plot(df_smooth['x'].values, df_smooth['y'].values, 
         'k--', label='Trayectoria Suavizada (Spline)', linewidth=2)

# C) Trayectoria Robot (Base Link) -> Línea Azul
plt.plot(df_real['x_robot'].values, df_real['y_robot'].values, 
         'b-', label='Robot Real (Base Link)', alpha=0.6)

# D) Punto Descentrado -> Línea Roja Fina
plt.plot(df_real['x_desc'].values, df_real['y_desc'].values, 
         'r-', label='Punto Descentrado (Control)', linewidth=1)

# --- NUEVO: MARCAR INICIO Y FINAL ---
# Usamos el primer y ultimo punto de la trayectoria original para marcar Inicio/Meta
if not df_orig.empty:
    # Inicio: Estrella Amarilla con borde negro
    start_x = df_orig['x'].values[0]
    start_y = df_orig['y'].values[0]
    plt.scatter(start_x, start_y, c='yellow', marker='*', s=200, edgecolors='black', label='Inicio', zorder=5)

    # Meta: Cuadrado Negro
    end_x = df_orig['x'].values[-1]
    end_y = df_orig['y'].values[-1]
    plt.scatter(end_x, end_y, c='black', marker='s', s=100, label='Meta Final', zorder=5)
# ------------------------------------

# Estética
plt.title('Trayectoria seguida')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.legend()
plt.grid(True)
plt.axis('equal') 

print("Generando gráfico...")
plt.show()
