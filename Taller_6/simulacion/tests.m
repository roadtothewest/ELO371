% =========================================================================
% Script Principal - Taller 6: Ascensor duple diagonal
% Sistema electrico - rotacional - traslacion
%   Vs(t) -> convertidor AC-AC -> rectificador 3F -> motor CC -> ejes ->
%   rueda -> dos cabinas contrapesadas (cable rigido, sin roce seco)
% =========================================================================
clear; clc; close all;
aqui = fileparts(mfilename('fullpath'));
if isempty(aqui), aqui = pwd; end
cd(aqui);

nombre_modelo = 'ascensor_duple_t6';
MODO      = 'simulink';  % 'simulink' o 'ecuacion' (integracion directa)
VERIFICAR = true;        % integra tambien la ecuacion de estado y compara
if exist('OCTAVE_VERSION', 'builtin')          % Octave no tiene Simulink
    MODO = 'ecuacion'; VERIFICAR = false;
end
if strcmp(MODO, 'simulink')
    if ~exist(fullfile(aqui, [nombre_modelo '.slx']), 'file')
        construir_modelo_t6(nombre_modelo);
    end
    load_system(nombre_modelo);
end

%% 2. Valores asignados (Tabla 1 del informe: ver parametros_t6.m)
% Cabina vacia 1200 kg, capacidad 8 pasajeros x 75 kg = 600 kg.
% Carga media del Taller 2 (4 pasajeros): 1500 kg.
m_vacia = 1200; m_media = 1500; m_llena = 1800;

% Viaje de la cabina A desde la estacion inferior hasta la superior:
% curva S de 8 s, crucero a 2 m/s y frenado de 20 s (sin regeneracion).
perfil = struct('vc', 2, 'Ta', 8, 'Td', 20, 'L', 100);

ensayos = {
    'fig_ensayo1',  'Ensayo 1: cargas equilibradas (pregunta 3)', m_media, m_media
    'fig_ensayo2a', 'Ensayo 2a: cabina llena sube (pregunta 4)',  m_llena, m_vacia
    'fig_ensayo2b', 'Ensayo 2b: cabina llena baja (pregunta 4)',  m_vacia, m_llena
    };

%% 3 y 4. Ensayos a velocidad constante
R = cell(size(ensayos, 1), 1);
KC = cell(size(ensayos, 1), 1);
for k = 1:size(ensayos, 1)
    p  = parametros_t6(ensayos{k, 3}, ensayos{k, 4});
    ff = prealimentacion_t6(p, perfil);
    tf = ff.T + 6;
    fprintf('\n%s  (mA = %g kg, mB = %g kg)\n', ensayos{k, 2}, p.mA, p.mB);
    tic;
    r = simular_t6(nombre_modelo, p, ff, tf, MODO);
    fprintf('  simulado (%s) en %.1f s\n', MODO, toc);

    if VERIFICAR
        % El diagrama debe reproducir la ecuacion de estado integrada aparte
        re = simular_t6(nombre_modelo, p, ff, tf, 'ecuacion');
        dv = max(abs(interp1(re.t, re.v, r.t) - r.v));
        dU = max(abs(interp1(re.t, re.U, r.t) - r.U));
        di = max(abs(interp1(re.t, re.ia, r.t) - r.ia));
        fprintf('  verificacion Simulink vs ecuacion: |dv| = %.2e m/s, |dU| = %.2e V, |dia| = %.2e A\n', ...
            dv, dU, di);
    end

    % Resumen numerico para el informe
    kc = r.t > ff.Ta + 4 & r.t < ff.Ta + ff.Tc - 2;     % tramo de crucero
    ka = find(r.xA >= p.xS, 1);
    kv = r.t <= ff.T & r.xA <= p.xS;                   % dentro del recorrido
    tm = ff.Ta + ff.Tc/2;                              % mitad del crucero
    fprintf('  requerido en crucero: U* = %.2f V, ia* = %.2f A, Vs = %.1f V rms\n', ...
        interp1(ff.t, ff.U, tm), interp1(ff.t, ff.i, tm), interp1(ff.t, ff.Vef, tm));
    fprintf('  crucero: v = %.4f m/s (%.4f a %.4f)\n', mean(r.v(kc)), min(r.v(kc)), max(r.v(kc)));
    fprintf('           U = %.1f V (%.1f a %.1f), ia = %.2f A (%.2f a %.2f), iD max = %.0f A\n', ...
        mean(r.U(kc)), min(r.U(kc)), max(r.U(kc)), mean(r.ia(kc)), ...
        min(r.ia(kc)), max(r.ia(kc)), max(r.iD(kc)));
    fprintf('  ia en todo el viaje: %.2f a %.2f A; ia* minima requerida: %.2f A\n', ...
        min(r.ia), max(r.ia), min(ff.i));
    if isempty(ka)
        fprintf('  llegada: se detiene en xA = %.3f m sin tocar el tope\n', r.xA(end));
    else
        fprintf('  llegada al tope a %.3f m/s (t = %.1f s), penetracion maxima %.0f mm\n', ...
            r.v(ka), r.t(ka), 1e3*(max(r.xA) - p.xS));
    end
    fprintf('  tension de cables en el recorrido: FA %.0f a %.0f N, FB %.0f a %.0f N\n', ...
        min(r.FA(kv)), max(r.FA(kv)), min(r.FB(kv)), max(r.FB(kv)));

    graficar_ensayo_t6(r, p, ff, ensayos{k, 2}, ensayos{k, 1});
    R{k} = r; KC{k} = kc;
end

%% Ley de regimen permanente del voltaje en el motor (preguntas 3 y 4)
% U = A*v + B*dm, con dm = mA - mB (positivo si sube la cabina mas pesada)
p   = parametros_t6();
vc  = perfil.vc;
cm  = p.b1 + (p.b2 + 2*p.gamma*p.r^2)/p.N^2;
A   = p.Ka*p.N/p.r + p.Ra*cm*p.N/(p.Ka*p.r);
B   = p.Ra*p.r*p.g*sin(p.theta)/(p.Ka*p.N);
dm  = linspace(-900, 900, 361)';
i_s = (cm*p.N/p.r*vc + p.r*dm*p.g*sin(p.theta)/p.N)/p.Ka;
U_s = A*vc + B*dm;
Vs_s = rectificador_promedio(p, U_s, i_s)/(p.a*sqrt(2));
dm_lim = -cm*p.N^2*vc/(p.r^2*p.g*sin(p.theta));      % ia = 0 en crucero
fprintf('\nLey de regimen: U = %.2f v + %.5f dm  [V]\n', A, B);
fprintf('  equilibradas (v = %g m/s): U = %.2f V, ia = %.2f A\n', vc, A*vc, interp1(dm, i_s, 0));
fprintf('  limite sin regeneracion: dm = %.0f kg (cabina mas pesada bajando)\n', dm_lim);

fig = figure('Name', 'Regimen permanente', 'Color', 'w', 'Position', [80 80 1200 330]);
dm_ens = [0, m_llena - m_vacia, m_vacia - m_llena];
subplot(1, 3, 1);
plot(dm, U_s, 'b', 'LineWidth', 1.5); hold on;
for k = 1:3
    plot(dm_ens(k), mean(R{k}.U(KC{k})), 'ko', 'MarkerFaceColor', 'y');
end
grid on; xlabel('\Delta m = m_A - m_B [kg]'); ylabel('U [V]');
title('(a) voltaje en el motor, v = 2 m/s');
subplot(1, 3, 2);
plot(dm, i_s, 'r', 'LineWidth', 1.5); hold on;
for k = 1:3
    plot(dm_ens(k), mean(R{k}.ia(KC{k})), 'ko', 'MarkerFaceColor', 'y');
end
plot(dm([1 end]), [0 0], 'k:', [dm_lim dm_lim], [min(i_s) max(i_s)], 'k--');
grid on; xlabel('\Delta m = m_A - m_B [kg]'); ylabel('i_a [A]');
title('(b) corriente de armadura');
subplot(1, 3, 3);
plot(dm, Vs_s, 'k', 'LineWidth', 1.5); hold on;
plot(dm([1 end]), [p.Vn p.Vn], 'k:');
grid on; xlabel('\Delta m = m_A - m_B [kg]'); ylabel('V_s [V rms]');
title('(c) tension monofasica requerida');
exportar_figura_t6(fig, 'fig_regimen');
