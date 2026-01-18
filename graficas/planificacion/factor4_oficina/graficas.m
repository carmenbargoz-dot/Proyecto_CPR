%% SCRIPT OCTAVE: Comparativa Barras (Leyenda Única)
clear; clc; close all;

%% 1. CONFIGURACIÓN
% Archivos que están en la misma carpeta que el script
archivos = { ...
    'factor2_oficina_A_star_planning_metrics.txt', ...
    'factor2_oficina_theta_star_planning_metrics.txt', ...
    'factor2_oficina_theta_star2_planning_metrics.txt' ...
};

leyenda_algoritmos = {'A*', 'Theta*', 'Theta* Safety'};

nombres_metricas = {
    'Tiempo de Planificación (s)', ...
    'Longitud del Camino (m)', ...
    'Número de Puntos (Waypoints)', ...
    'Distancia Media a Obstáculos (m)'
};

% Colores: Azul, Rojo, Verde
colores_barras = {'b', 'r', 'g'}; 

%% 2. LECTURA DE DATOS
printf('Directorio actual: %s\n', pwd);
num_archivos = length(archivos);
datos_procesados = []; 

for k = 1:num_archivos
    if exist(archivos{k}, 'file') ~= 2
        error('ERROR: No encuentro "%s".', archivos{k});
    end
    % Lee saltando header y timestamp
    datos_procesados(:, :, k) = dlmread(archivos{k}, ';', 1, 1);
end

%% 3. GENERACIÓN DE BARRAS (SUBPLOT)
x = 1:size(datos_procesados, 1);

fig = figure('Name', 'Comparativa Barras', 'NumberTitle', 'off', 'Visible', 'on');
set(fig, 'Position', [100, 100, 1200, 800]); 

for col = 1:4
    subplot(2, 2, col);
    hold on; grid on; box on;
    
    % --- PREPARAR DATOS ---
    datos_grafica = [];
    for k = 1:num_archivos
        datos_grafica(:, k) = datos_procesados(:, col, k);
    end
    
    % --- DIBUJAR BARRAS ---
    h = bar(x, datos_grafica);
    
    % --- COLOREAR ---
    for k = 1:num_archivos
        set(h(k), 'FaceColor', colores_barras{k});
    end
    
    % --- DECORACIÓN ---
    title(nombres_metricas{col}, 'FontSize', 10, 'FontWeight', 'bold');
    xlabel('Caso');
    xlim([0.5, length(x)+0.5]);
    set(gca, 'XTick', x); 
    
    % --- LEYENDA ÚNICA (EL CAMBIO ESTÁ AQUÍ) ---
    if col == 1
        % Solo creamos la leyenda si estamos en la primera columna/gráfica
        % 'Location', 'best' busca el sitio donde menos tape
        % 'Location', 'NorthWest' la fuerza arriba a la izquierda
        % 'Orientation', 'horizontal' la pone a lo largo
        legend(h, leyenda_algoritmos, 'Location', 'best', 'FontSize', 8);
    end
    
    hold off;
end

%% 4. GUARDAR
nombre_imagen = 'comparativa_barras_oficina.png';
printf('Guardando "%s"...\n', nombre_imagen);
print(fig, nombre_imagen, '-dpng', '-r300');

