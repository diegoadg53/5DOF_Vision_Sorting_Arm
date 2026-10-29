%% Comparación de métodos de cinemática inversa (Figuras [G] y [G2] de la sección 8.1)
% Caso 1: objetivo alcanzable. Compara la convergencia del error de posición.
% Caso 2: objetivo fuera del alcance. Compara el tamaño de los incrementos
%         articulares e incluye el límite de paso que usa el firmware.
% Requiere el Robotics Toolbox de Peter Corke (SerialLink).
% Guarda Fig_convergencia_IK.png y Fig_objetivo_inalcanzable_IK.png en la
% misma carpeta del script.
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
robot = SerialLink(L, 'name', 'Brazo', 'base', T_B0, 'tool', T_herr);

%% 2. Parámetros comunes
q0     = zeros(5, 1);   % configuración inicial: HOME
N_iter = 100;           % número de iteraciones
tol    = 1e-4;          % tolerancia de error, en mm
K      = 0.5;           % ganancia de los métodos clásicos
lambda = 0.1;           % factor de amortiguamiento, en mm
colores = [0.80 0.10 0.15; 0.15 0.35 0.80; 0.10 0.60 0.20; 0.55 0.25 0.65];
carpeta = fileparts(mfilename('fullpath'));
if isempty(carpeta), carpeta = pwd; end

%% 3. Caso 1: objetivo alcanzable
od1 = [300; 150; 200];
metodos1 = {'pinv', 'dls', 'trans'};
nombres1 = {'Pseudoinversa', 'Mínimos cuadrados amortiguados', 'Transpuesta del jacobiano'};
[err1, paso1] = simular(robot, q0, od1, metodos1, N_iter, tol, K, lambda);

fig1 = figure('Color', 'w', 'Position', [100 100 900 480]);
estilos = {'-', '--', '-.'};
for m = 1:numel(metodos1)
    semilogy(1:N_iter, max(err1(:, m), 1e-6), estilos{m}, 'Color', colores(m, :), 'LineWidth', 2);
    hold on;
end
grid on;
xlim([1 30]);
xlabel('Iteración');
ylabel('Norma del error de posición (mm)');
legend(nombres1, 'Location', 'northeast');
set(gca, 'FontSize', 11);
exportgraphics(fig1, fullfile(carpeta, 'Fig_convergencia_IK.png'), 'Resolution', 300);

fprintf('Caso 1: objetivo alcanzable (%.0f, %.0f, %.0f) mm\n', od1);
for m = 1:numel(metodos1)
    k1 = find(err1(:, m) < 1, 1);
    fprintf('  %-32s error inicial %.1f mm, error < 1 mm en la iteración %d, error en la iteración 10: %.3f mm, paso máximo %.1f°\n', ...
            nombres1{m}, err1(1, m), k1, err1(10, m), max(paso1(:, m)));
end

%% 4. Caso 2: objetivo fuera del alcance
od2 = [800; 0; 150];
metodos2 = {'pinv', 'dls', 'dls_limitado'};
nombres2 = {'Pseudoinversa', 'Mínimos cuadrados amortiguados', ...
            'Mínimos cuadrados amortiguados con límite de paso'};
[err2, paso2] = simular(robot, q0, od2, metodos2, N_iter, tol, K, lambda);

fig2 = figure('Color', 'w', 'Position', [100 100 1300 520]);
tl = tiledlayout(fig2, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax1 = nexttile(tl);
for m = 1:numel(metodos2)
    plot(ax1, 1:N_iter, err2(:, m), 'Color', colores(m, :), 'LineWidth', 1.6);
    hold(ax1, 'on');
end
grid(ax1, 'on');
xlabel(ax1, 'Iteración');
ylabel(ax1, 'Norma del error de posición (mm)');
title(ax1, '(a) Error de posición', 'FontWeight', 'normal');
set(ax1, 'FontSize', 11);

ax2 = nexttile(tl);
for m = 1:numel(metodos2)
    semilogy(ax2, 1:N_iter, max(paso2(:, m), 1e-3), 'Color', colores(m, :), 'LineWidth', 1.6);
    hold(ax2, 'on');
end
grid(ax2, 'on');
ylim(ax2, [1 1e4]);
xlabel(ax2, 'Iteración');
ylabel(ax2, 'Incremento articular máximo (°)');
title(ax2, '(b) Tamaño de cada paso', 'FontWeight', 'normal');
set(ax2, 'FontSize', 11);
lgd = legend(ax2, nombres2, 'Orientation', 'horizontal', 'FontSize', 11);
lgd.Layout.Tile = 'south';
lgd.Box = 'off';
exportgraphics(fig2, fullfile(carpeta, 'Fig_objetivo_inalcanzable_IK.png'), 'Resolution', 300);

fprintf('\nCaso 2: objetivo fuera del alcance (%.0f, %.0f, %.0f) mm\n', od2);
for m = 1:numel(metodos2)
    fprintf('  %-50s error mínimo %.1f mm, error final %.1f mm, paso máximo %.1f°\n', ...
            nombres2{m}, min(err2(:, m)), err2(end, m), max(paso2(:, m)));
end
fprintf('\nFiguras guardadas en %s\n', carpeta);

%% Funciones locales
function [errores, pasos] = simular(robot, q0, od, metodos, N_iter, tol, K, lambda)
    errores = zeros(N_iter, numel(metodos));
    pasos   = zeros(N_iter, numel(metodos));
    for m = 1:numel(metodos)
        q = q0;
        for k = 1:N_iter
            T  = robot.fkine(q').T;
            e  = od - T(1:3, 4);
            errores(k, m) = norm(e);
            if norm(e) <= tol
                continue;
            end
            J  = robot.jacob0(q');
            Jv = J(1:3, :);
            dq = paso(metodos{m}, Jv, e, K, lambda);
            pasos(k, m) = rad2deg(max(abs(dq)));
            q = q + dq;
        end
    end
end

function dq = paso(metodo, Jv, e, K, lambda)
    switch metodo
        case 'pinv'
            dq = pinv(Jv) * (K * e);
        case 'dls'
            dq = Jv' * ((Jv * Jv' + lambda^2 * eye(3)) \ (K * e));
        case 'trans'
            JJ = Jv * Jv';
            alfa = (e' * JJ * e) / (e' * JJ * JJ * e + 1e-6);
            dq = alfa * Jv' * (K * e);
        case 'dls_limitado'
            % Mismo límite de paso del firmware: el incremento se escala
            % para no superar 0.12 rad y luego se aplica una ganancia de 0.7.
            dq = Jv' * ((Jv * Jv' + lambda^2 * eye(3)) \ e);
            mayor = max(abs(dq));
            g = 0.7;
            if mayor > 0.12
                g = 0.7 * 0.12 / mayor;
            end
            dq = g * dq;
    end
end
