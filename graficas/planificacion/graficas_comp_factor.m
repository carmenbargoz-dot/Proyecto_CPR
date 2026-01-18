%% SCRIPT OCTAVE: Comparativa por Factor de Downsample (Barras)
clear; clc; close all;

%% 1. CONFIGURACIÓN
% Pon aquí tus 3 archivos correspondientes a cada factor
% Orden: [Factor 2, Factor 4, Factor 6]
archivos = { ...
    'factor2_oficina_theta_star2_planning_metrics.txt', ...  % Archivo del Factor 2
    'factor4_oficina_Theta_star2_planning_metrics.txt', ...  % Archivo del Factor 4
    'factor6_oficina_theta_star2_planning_metrics.txt' ...   % Archivo del Factor 6
};

% Leyenda para identificar cada barra
leyenda_factores = {'Factor 2 (Alta Res)', 'Factor 4 (Media)', 'Factor 6 (Baja Res)'};

% Métricas (Columnas 2, 3, 4 y 5)
nombres_metricas = {
    'Tiempo de Planificación (s)', ...
    'Longitud del Camino (m)', ...
    'Número de Puntos (Waypoints)', ...
    'Distancia Media a Obstáculos (m)'
};

% Colores: Azul (F2), Verde (F4), Rojo (F6) para indicar "pérdida de resolución"
colores_barras = {'b', 'g', 'r'}; 

%% 2. LECTURA DE DATOS
printf('Directorio actual: %s\n', pwd);
num_archivos = length(archivos);
datos_procesados = []; 

for k = 1:num_archivos
    if exist(archivos{k}, 'file') ~= 2
        warning('AVISO: No encuentro "%s". Se usaran ceros si no lo corriges.', archivos{k});
        % Si no existe, rellena con ceros para no romper el script (opcional)
        % datos_procesados(:, :, k) = zeros(20, 4); 
        error('ERROR: Falta el archivo "%s". Verifícalo.', archivos{k});
    end
    
    % Leer saltando header y timestamp
    datos_procesados(:, :, k) = dlmread(archivos{k}, ';', 1, 1);
end

%% 3. GENERACIÓN DE BARRAS (SUBPLOT)
x = 1:size(datos_procesados, 1);

fig = figure('Name', 'Comparativa por Factor', 'NumberTitle', 'off', 'Visible', 'on');
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
    
    % --- LEYENDA ÚNICA EN LA PRIMERA GRÁFICA ---
    if col == 1
        legend(h, leyenda_factores, 'Location', 'best', 'FontSize', 8);
    end
    
    hold off;
end

%% 4. GUARDAR
nombre_imagen = 'comparativa_factores_downsample.png';
printf('Guardando "%s"...\n', nombre_imagen);
print(fig, nombre_imagen, '-dpng', '-r300');

printf('¡Gráfica de factores generada!\n');
