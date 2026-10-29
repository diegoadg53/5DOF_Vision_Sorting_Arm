%% Eje ciego de rotación del brazo de 5 GDL (Figura [H] de la sección 8.1)
% Grafica el brazo en HOME y en una configuración de prueba, con el eje de
% rotación inalcanzable (eje ciego) y los ejes de la herramienta.
% Requiere el Robotics Toolbox de Peter Corke (SerialLink).
% Guarda Fig_eje_ciego.png en la misma carpeta del script.
clear; clc; close all;

%% 1. Modelo DH, base y herramienta (iguales al firmware y al README)
L(1) = Link('d', 0,       'a', 0,     'alpha',  pi/2, 'offset', pi/2);
L(2) = Link('d', 315.557, 'a', 47.45, 'alpha',  pi/2);
L(3) = Link('d', 0,       'a', 307,   'alpha',  pi);
L(4) = Link('d', 0,       'a', 0,     'alpha', -pi/2);
L(5) = Link('d', 115.14,  'a', 0,     'alpha',  pi);

R_B0 = rotx(-pi/2) * rotz(-pi/2);   % igual a [0 1 0; 0 0 1; 1 0 0]
p_B0 = [-76; 0; 102.465];           % origen del marco 0 en la base {B}, en mm
T_B0 = [R_B0, p_B0; 0 0 0 1];
T_herr = transl(0, 0, -49);         % TCP a 49 mm sobre el eje Z del último marco

%% 2. Configuraciones a graficar (en radianes)
configs = {zeros(1, 5), [-0.4, -0.5, -0.3, 0, 0.1]};
etiquetas = {'(a) HOME', '(b) Configuración de prueba'};
espacio = [-200 450 -350 250 -60 520];   % límites de los ejes, en mm
escala = 150;                            % longitud de las flechas, en mm

%% 3. Cálculo y gráfica
fig = figure('Color', 'w', 'Position', [100 100 1350 600]);
for k = 1:2
    q = configs{k};
    % Un robot por panel, con nombre distinto, para que el Toolbox no
    % anime el mismo dibujo en ambos paneles.
    robot = SerialLink(L, 'name', sprintf('Brazo_%d', k), 'base', T_B0, 'tool', T_herr);
    [b, b_izq, T] = eje_ciego(robot, q);

    subplot(1, 2, k);
    robot.plot(q, 'workspace', espacio, 'noname', 'notiles', 'noshadow', 'nowrist', 'jointdiam', 1.5);
    hold on;
    p = T(1:3, 4);
    % El eje ciego se dibuja un poco más largo para que se distinga cuando
    % coincide con el eje X. El eje Z de la herramienta apunta hacia la
    % muñeca, así que se dibuja en el sentido de aproximación (-Z) para que
    % no quede escondido dentro del brazo.
    hb = flecha(p, b * escala * 1.3, [0.80 0.10 0.15], 3.0);
    hx = flecha(p, T(1:3, 1) * escala, [0.10 0.60 0.20], 2.0);
    hy = flecha(p, T(1:3, 2) * escala, [0.90 0.65 0.05], 2.0);
    hz = flecha(p, -T(1:3, 3) * escala, [0.15 0.35 0.80], 2.0);
    view(-35, 22);
    xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
    title(etiquetas{k}, 'FontWeight', 'normal');

    angulos = acosd(abs(T(1:3, 1:3)' * b));
    fprintf('%s\n', etiquetas{k});
    fprintf('  TCP = (%.2f, %.2f, %.2f) mm\n', p);
    fprintf('  Eje ciego = (%.3f, %.3f, %.3f)\n', b);
    fprintf('  Ángulo con los ejes X, Y y Z de la herramienta: %.1f°, %.1f°, %.1f°\n', angulos);
    fprintf('  Diferencia con el método del espacio nulo izquierdo: %.4f°\n\n', ...
            acosd(min(1, abs(dot(b, b_izq)))));
end

lgd = legend([hb hx hy hz], {'Eje ciego', 'Eje X de la herramienta', ...
              'Eje Y de la herramienta', 'Sentido de aproximación (-Z de la herramienta)'}, ...
              'Orientation', 'horizontal', 'FontSize', 11);
lgd.Position = [0.25 0.02 0.5 0.05];
lgd.Box = 'off';

%% 4. Exportar
carpeta = fileparts(mfilename('fullpath'));
if isempty(carpeta), carpeta = pwd; end
exportgraphics(fig, fullfile(carpeta, 'Fig_eje_ciego.png'), 'Resolution', 300);
fprintf('Figura guardada en %s\n', fullfile(carpeta, 'Fig_eje_ciego.png'));

%% Funciones locales
function [b, b_izq, T] = eje_ciego(robot, q)
    % Método del firmware: rotaciones alcanzables con el TCP fijo.
    J  = robot.jacob0(q);
    Jv = J(1:3, :);
    Jw = J(4:6, :);
    N  = null(Jv);              % movimientos articulares que no desplazan el TCP
    W  = Jw * N;                % rotaciones que esos movimientos generan
    [U, ~, ~] = svd(W);
    b  = U(:, 3);               % dirección perpendicular a range(W): eje ciego
    % Método original del script: espacio nulo izquierdo del jacobiano.
    n  = null(J');
    b_izq = n(4:6, 1) / norm(n(4:6, 1));
    % Signos solo para que las flechas se vean en la misma dirección.
    T = robot.fkine(q).T;
    if dot(b, T(1:3, 1)) < 0, b = -b; end
    if dot(b_izq, b) < 0, b_izq = -b_izq; end
end

function h = flecha(origen, vector, color, grosor)
    h = quiver3(origen(1), origen(2), origen(3), vector(1), vector(2), vector(3), ...
                'Color', color, 'LineWidth', grosor, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
end
