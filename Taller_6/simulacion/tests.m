% =========================================================================
% Script Principal - Taller 6: Ascensor duple diagonal
% Sistema electrico - rotacional - traslacion
%   Vs(t) -> convertidor AC-AC -> rectificador 3F -> motor CC -> ejes ->
%   rueda -> dos cabinas contrapesadas (cable rigido, sin roce seco)
%
% Archivo unico: solo necesita ascensor_duple_t6.slx en la misma carpeta.
% Si el .slx no existe, lo genera construir_modelo_t6.m (una sola vez).
% Todas las funciones (parametros, prealimentacion, ecuacion de estado,
% graficos) estan al final de este archivo.
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

%% 2. Valores asignados (Tabla 1 del informe: ver parametros_t6 al final)
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

% =========================================================================
% FUNCIONES LOCALES
% =========================================================================

function p = parametros_t6(mA, mB)
% =========================================================================
% PARAMETROS_T6  Valores asignados a los elementos del sistema - Taller 6
%   p = parametros_t6(mA, mB) entrega la estructura de parametros con la
%   masa de la cabina A (la que sube) y de la cabina B (la que baja).
%   Sin argumentos usa cargas equilibradas: 1500 kg por cabina (Taller 2).
% =========================================================================
if nargin < 2
    mA = 1500; mB = 1500;
end

%% Fuente monofasica y convertidor AC-AC (1 fase -> 3 fases)
p.f  = 50;            % [Hz]   Frecuencia de la red
p.Vn = 220;           % [V rms] Tension nominal monofasica
p.a  = 1.5;           % [-]    Razon del convertidor (amplitud de fase de salida / entrada)

%% Rectificador trifasico de media onda (3 diodos) con condensador de bus
p.Rs = 0.2;           % [Ohm]  Resistencia serie por fase (devanado + diodo)
p.C  = 10e-3;         % [F]    Condensador del bus de CC

%% Motor de CC (identico al Taller 3)
p.Ra = 0.5;           % [Ohm]    Resistencia de armadura
p.La = 8e-3;          % [H]      Inductancia de armadura
p.Ka = 2.6;           % [Vs/rad] Constante de maquina

%% Sistema de potencia rotacional (identico al Taller 3)
p.J1 = 0.35;          % [Nms^2]   Inercia del rotor y eje del motor
p.b1 = 0.015;         % [Nms/rad] Roce viscoso del eje del motor
p.K1 = 2e-4;          % [rad/Nm]  Flexibilidad torsional del eje del motor
p.N  = 40;            % [-]       Razon de reduccion del juego de engranajes
p.J2 = 3;             % [Nms^2]   Inercia del engranaje secundario y su eje
p.b2 = 10;            % [Nms/rad] Roce viscoso del eje secundario
p.K2 = 2e-6;          % [rad/Nm]  Flexibilidad torsional del eje secundario
p.Jp = 50;            % [Nms^2]   Inercia de la rueda
p.r  = 0.5;           % [m]       Radio de la rueda

%% Carga: dos cabinas del Taller 2 (sin roce seco ni lluvia)
p.mA    = mA;         % [kg]   Cabina A (sube)
p.mB    = mB;         % [kg]   Cabina B (baja)
p.g     = 9.81;       % [m/s^2]
p.theta = pi/6;       % [rad]  Inclinacion del riel (30 grados)
p.gamma = 800;        % [Ns/m] Roce viscoso de cada cabina con su riel
p.KS    = 1.5e5;      % [N/m]  Rigidez del amortiguador superior
p.KB    = 1.5e5;      % [N/m]  Rigidez del amortiguador inferior
p.gS    = 29200;      % [Ns/m] Roce del amortiguador superior
p.gB    = 29200;      % [Ns/m] Roce del amortiguador inferior
p.xS    = 100;        % [m]    Largo del riel

%% Constantes agrupadas (punto 11 del informe)
p.Jeq = p.Jp + (p.mA + p.mB)*p.r^2;     % rueda + ambas cabinas reflejadas
p.Ke  = p.KS + p.KB;                    % resortes en contacto simultaneo
p.ge  = p.gS + p.gB;                    % roces en contacto simultaneo
p.Fg  = (p.mA - p.mB)*p.g*sin(p.theta); % desequilibrio de peso sobre el cable

%% Simulacion
p.x0     = zeros(10, 1);  % [U ia wm Mk1 w2 Mk2 wp FkS FkI xA] en reposo
p.MaxStep = 1e-4;         % [s] paso maximo (resolver los pulsos de diodo)
p.Ts_log  = 2e-4;         % [s] periodo de registro de las senales
end

function ff = prealimentacion_t6(p, perfil)
% =========================================================================
% PREALIMENTACION_T6  Excitacion Vs(t) para un perfil de velocidad deseado.
%   ff = prealimentacion_t6(p, perfil) con perfil.vc [m/s], perfil.Ta y
%   perfil.Td [s] (rampas S de aceleracion y frenado) y perfil.L [m].
%
%   Dinamica inversa sobre el MODELO PROMEDIO del informe: ejes rigidos
%   (se eliminan los modos de torsion, mucho mas rapidos que el viaje) y
%   rectificador promediado (se elimina el rizado de 150 Hz). Se obtienen:
%     ff.U   voltaje requerido en el motor         [V]
%     ff.i   corriente de armadura requerida       [A]
%     ff.Vm  amplitud de fase requerida del conv.  [V]
%     ff.Vef valor eficaz de Vs(t) a aplicar       [V rms]
% =========================================================================
dt = 5e-3;
vc = perfil.vc; Ta = perfil.Ta; Td = perfil.Td;
Tc = (perfil.L - vc*(Ta + Td)/2)/vc;          % duracion del crucero
T  = Ta + Tc + Td;
t  = (0:dt:T + 10)';

% Perfil de velocidad S (polinomio quintico: v, a y jerk nulos en extremos)
u1 = (t - 0)/Ta;  u2 = (t - Ta - Tc)/Td;
v = vc*(s5(u1) - s5(u2));
a = vc*(ds5(u1)/Ta - ds5(u2)/Td);
j = vc*(dds5(u1)/Ta^2 - dds5(u2)/Td^2);

% Modelo rigido referido al eje del motor
Jtot = p.J1 + (p.J2 + p.Jeq)/p.N^2;            % inercia total
cm   = p.b1 + (p.b2 + 2*p.gamma*p.r^2)/p.N^2;  % roce viscoso total
Mg   = p.r*p.Fg/p.N;                           % torque del desequilibrio
wm   = p.N/p.r*v;
Mm   = Jtot*p.N/p.r*a + cm*wm + Mg;
dMm  = Jtot*p.N/p.r*j + cm*p.N/p.r*a;
i    = Mm/p.Ka;
U    = p.Ra*i + p.La*dMm/p.Ka + p.Ka*wm;

% Rectificador promedio (punto 13 del informe): ID incluye la carga del
% condensador del bus para seguir la rampa de U.
ID = i + p.C*gradient(U, dt);
Vm = rectificador_promedio(p, U, ID);

ff.t = t; ff.v = v; ff.a = a; ff.U = U; ff.i = i; ff.ID = ID;
ff.Vm = Vm; ff.Vef = Vm/(p.a*sqrt(2));
ff.T = T; ff.Ta = Ta; ff.Tc = Tc; ff.Td = Td; ff.Jtot = Jtot; ff.cm = cm; ff.Mg = Mg;
end

function s = s5(u)
u = min(max(u, 0), 1);
s = 6*u.^5 - 15*u.^4 + 10*u.^3;
end

function s = ds5(u)
s = 30*u.^2.*(1 - u).^2;
s(u < 0 | u > 1) = 0;
end

function s = dds5(u)
s = 60*u - 180*u.^2 + 120*u.^3;
s(u < 0 | u > 1) = 0;
end

function Vm = rectificador_promedio(p, U, ID)
% RECTIFICADOR_PROMEDIO  Modelo promedio (estatico) del rectificador de 3 diodos.
%   Vm = rectificador_promedio(p, U, ID) entrega la amplitud de fase Vm que
%   el convertidor debe aplicar para que el bus quede en U entregando la
%   corriente media ID. Si U es casi constante en un periodo, cada diodo
%   conduce mientras Vm*cos(th) > U, es decir |th| < thc, y el balance de
%   corriente media de las tres fases es
%       ID = 3/(pi*Rs)*(Vm*sin(thc) - U*thc),   U = Vm*cos(thc)
%   de donde  tan(thc) - thc = pi*Rs*ID/(3*U).
%   Si ID <= 0 los diodos no conducen: basta Vm = U (borde de conduccion).
Vm = max(U, 0);
k  = (ID > 0) & (U > 0);
R  = pi*p.Rs*ID(k)./(3*U(k));
lo = zeros(size(R)); hi = (pi/2 - 1e-9)*ones(size(R));
for it = 1:60                   % biseccion vectorizada en (0, pi/2)
    th = (lo + hi)/2;
    f  = tan(th) - th - R;
    lo(f < 0) = th(f < 0);
    hi(f >= 0) = th(f >= 0);
end
Vm(k) = U(k)./cos((lo + hi)/2);
end

function r = simular_t6(mdl, p, ff, tf, modo)
% =========================================================================
% SIMULAR_T6  Ejecuta un ensayo y devuelve las senales registradas.
%   r = simular_t6(mdl, p, ff, tf, 'simulink')  usa el modelo Simulink
%   r = simular_t6(mdl, p, ff, tf, 'ecuacion')  integra directamente la
%       ecuacion de estado (RK4, paso fijo p.MaxStep); sirve para verificar
%       que el diagrama Simulink implementa la misma ecuacion.
%   ff es la prealimentacion (prealimentacion_t6) y tf el tiempo final.
% =========================================================================
switch modo
    case 'simulink'
        simIn = Simulink.SimulationInput(mdl);
        simIn = simIn.setVariable('p', p);
        simIn = simIn.setVariable('Vef_ts', timeseries(ff.Vef, ff.t));
        simIn = simIn.setModelParameter('StopTime', num2str(tf), ...
            'MaxStep', num2str(p.MaxStep));
        out = sim(simIn);
        r.t = out.U.Time(:);
        for s = {'Vs', 'U', 'iD', 'ia', 'wm', 'Mk1', 'Mk2', 'v', 'dwp', ...
                 'xA', 'FkS', 'FkI', 'Fc'}
            r.(s{1}) = squeeze(out.(s{1}).Data);
        end
        vabc = squeeze(out.vabc.Data);
        if size(vabc, 1) == 3 && size(vabc, 2) ~= 3, vabc = vabc.'; end
        r.vabc = vabc;

    case 'ecuacion'
        dt = p.MaxStep;
        n  = round(tf/dt);
        nq = round(1/(4*p.f)/dt);              % retardo T/4 en pasos
        dec = max(1, round(p.Ts_log/dt));
        % Vs en la grilla de medio paso que usa RK4 (indice 1 -> t = 0)
        th = (0:2*n)'*dt/2;
        Vs = sqrt(2)*interp1(ff.t, ff.Vef, th, 'linear', ff.Vef(end)).*sin(2*pi*p.f*th);
        vsd = @(k) (k > 2*nq)*Vs(max(k - 2*nq, 1));   % Vs(t - T/4), nulo antes
        x = p.x0(:);
        m = floor(n/dec) + 1;
        Xr = zeros(m, 10); IDr = zeros(m, 1); DWr = zeros(m, 1); VABC = zeros(m, 3);
        kr = 0;
        for k = 0:n
            i0 = 2*k + 1;
            v0 = convertidor_ac_ac(Vs(i0), vsd(i0), p.a);
            if mod(k, dec) == 0
                [~, iD, dwp] = ecuacion_estado_t6(x, v0, p);
                kr = kr + 1;
                Xr(kr, :) = x.'; IDr(kr) = iD; DWr(kr) = dwp; VABC(kr, :) = v0.';
            end
            if k == n, break; end
            vm = convertidor_ac_ac(Vs(i0 + 1), vsd(i0 + 1), p.a);
            v1 = convertidor_ac_ac(Vs(i0 + 2), vsd(i0 + 2), p.a);
            k1 = ecuacion_estado_t6(x, v0, p);
            k2 = ecuacion_estado_t6(x + dt/2*k1, vm, p);
            k3 = ecuacion_estado_t6(x + dt/2*k2, vm, p);
            k4 = ecuacion_estado_t6(x + dt*k3, v1, p);
            x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4);
        end
        Xr = Xr(1:kr, :);
        r.t = (0:kr-1)'*dec*dt;
        r.Vs = Vs(1:2*dec:2*dec*(kr-1)+1);
        r.vabc = VABC(1:kr, :);
        r.U = Xr(:, 1); r.ia = Xr(:, 2); r.wm = Xr(:, 3); r.Mk1 = Xr(:, 4);
        r.Mk2 = Xr(:, 6); r.v = p.r*Xr(:, 7); r.xA = Xr(:, 10);
        r.FkS = Xr(:, 8); r.FkI = Xr(:, 9);
        r.iD = IDr(1:kr); r.dwp = DWr(1:kr);
        s2 = r.xA > p.xS; s3 = r.xA < 0;
        r.Fc = p.Fg + 2*p.gamma*r.v + s2.*(r.FkS + p.ge*r.v) + s3.*(r.FkI + p.ge*r.v);
    otherwise
        error('modo desconocido: %s', modo);
end

% ---------------------------------------- salidas del punto 13 del informe
r.vA = r.v;  r.vB = -r.v;              % velocidades de las cabinas
r.xB = p.xS - r.xA;                     % posicion de la cabina B
a  = p.r*r.dwp;                         % aceleracion de la cabina A
gs = p.g*sin(p.theta);
s2 = r.xA > p.xS; s3 = r.xA < 0;
% Tension de cada cable (H6: deben ser positivas). Los amortiguadores en
% contacto simultaneo se reparten FkS y FkI segun su rigidez.
r.FA = p.mA*(a + gs) + p.gamma*r.v ...
     + s2.*(p.KS/p.Ke*r.FkS + p.gS*r.v) + s3.*(p.KB/p.Ke*r.FkI + p.gB*r.v);
r.FB = p.mB*(-a + gs) - p.gamma*r.v ...
     - s2.*(p.KB/p.Ke*r.FkS + p.gB*r.v) - s3.*(p.KS/p.Ke*r.FkI + p.gS*r.v);
end

function [dx, iD, dwp] = ecuacion_estado_t6(x, vabc, p)
% =========================================================================
% ECUACION_ESTADO_T6  Ecuacion de estado del Taller 6, dx/dt = f(x, u).
%   x    = [U; ia; wm; Mk1; w2; Mk2; wp; FkS; FkI; xA]  (punto 11 del informe)
%   vabc = tensiones de fase del convertidor AC-AC (entrada del rectificador)
%   Devuelve tambien la corriente de diodos iD y la aceleracion de la rueda.
% Es la misma ecuacion que implementa fila a fila el modelo Simulink; se usa
% para verificar el diagrama por integracion directa.
% =========================================================================
U   = x(1);  ia  = x(2);  wm  = x(3);  Mk1 = x(4);  w2 = x(5);
Mk2 = x(6);  wp  = x(7);  FkS = x(8);  FkI = x(9);  xA = x(10);

iD = rectificador_m3(vabc, U, p.Rs);

v  = p.r*wp;                       % velocidad de la cabina A (B = -v)
s2 = double(xA > p.xS);            % A en tope superior y B en tope inferior
s3 = double(xA < 0);               % A en tope inferior y B en tope superior
Fc = p.Fg + 2*p.gamma*v + s2*(FkS + p.ge*v) + s3*(FkI + p.ge*v);

dwp = (Mk2 - p.r*Fc)/p.Jeq;
dx = [ (iD - ia)/p.C;                       % (12.1) condensador del bus
       (U - p.Ra*ia - p.Ka*wm)/p.La;        % (12.2) armadura
       (p.Ka*ia - p.b1*wm - Mk1)/p.J1;      % (12.3) rotor J1
       (wm - p.N*w2)/p.K1;                  % (12.4) torsion K1
       (p.N*Mk1 - p.b2*w2 - Mk2)/p.J2;      % (12.5) eje secundario J2
       (w2 - wp)/p.K2;                      % (12.6) torsion K2
       dwp;                                 % (12.7) rueda + cabinas (Jeq)
       p.Ke*s2*v;                           % (12.8) resortes tope superior
       p.Ke*s3*v;                           % (12.9) resortes tope inferior
       v ];                                 % (12.10) posicion de la cabina A
end

function v = convertidor_ac_ac(vs, vs_d, a)
% CONVERTIDOR_AC_AC  Convertidor ideal monofasico -> trifasico.
%   vs   : tension de entrada Vs(t), tomada como componente alfa
%   vs_d : Vs(t - T/4), la misma tension retrasada 90 grados (componente beta)
%   a    : razon de amplitud del convertidor
%   v    : tensiones de fase [va; vb; vc] (secuencia positiva, neutro comun)
% Es la transformada inversa de Clarke escalada por a. El mismo codigo va en
% el bloque MATLAB Function "Convertidor_AC_AC" (construir_modelo_t6.m).
v = a*[vs; -0.5*vs + (sqrt(3)/2)*vs_d; -0.5*vs - (sqrt(3)/2)*vs_d];
end

function iD = rectificador_m3(v, U, Rs)
% RECTIFICADOR_M3  Rectificador trifasico de media onda (3 diodos, catodo comun).
%   v  : tensiones de fase [va; vb; vc] respecto del neutro
%   U  : tension del condensador del bus de CC (estado)
%   Rs : resistencia serie de cada fase
%   iD : corriente total entregada por los diodos al bus
% Cada diodo es ideal en serie con Rs: conduce solo si su fase supera a U.
% El mismo codigo va en el bloque MATLAB Function "Rectificador".
iD = 0;
for k = 1:3
    if v(k) > U
        iD = iD + (v(k) - U)/Rs;
    end
end
end

function graficar_ensayo_t6(r, p, ff, titulo, archivo)
% GRAFICAR_ENSAYO_T6  Figura de un ensayo (seis paneles) y exportacion a PDF.
fig = figure('Name', titulo, 'Color', 'w', 'Position', [60 60 1200 620]);
tf = r.t(end);
gris = [0.55 0.55 0.55];

% (a) Voltaje en el motor: senal rectificada, su promedio y el requerido
subplot(2, 3, 1);
Np = max(1, round(1/(3*p.f)/(r.t(2) - r.t(1))));       % un periodo de rizado
plot(r.t, r.U, 'Color', [0.65 0.78 0.95]); hold on;
plot(r.t, movmean(r.U, Np), 'b', 'LineWidth', 1.2);
plot(ff.t, ff.U, 'k--', 'LineWidth', 1);
xlim([0 tf]); grid on;
xlabel('t [s]'); ylabel('U [V]');
title('(a) voltaje en el motor');
legend('U(t)', 'promedio', 'U^* requerido', 'Location', 'south');

% (b) Detalle de tres periodos de red en crucero: rectificacion de 3 pulsos
subplot(2, 3, 2);
t0 = round(ff.Ta + ff.Tc/2);                          % mitad del crucero
k = r.t >= t0 & r.t <= t0 + 3/p.f;
tt = (r.t(k) - t0)*1e3;
plot(tt, r.vabc(k, :), 'Color', gris); hold on;
plot(tt, max(r.vabc(k, :), [], 2), 'k--');             % salida sin condensador
plot(tt, r.U(k), 'b', 'LineWidth', 1.5);
yl = max(abs(r.vabc(k, 1)))*1.1;
ylim([-yl yl]); xlim([0 3e3/p.f]); grid on;
xlabel('t - t_0 [ms]'); ylabel('[V]');
title(sprintf('(b) detalle en crucero, t_0 = %.0f s', t0));

% (c) Velocidad de ambas cabinas
subplot(2, 3, 3);
plot(r.t, r.vA, 'b', 'LineWidth', 1.2); hold on;
plot(r.t, r.vB, 'r', 'LineWidth', 1.2);
plot(ff.t, ff.v, 'k--', ff.t, -ff.v, 'k--');
xlim([0 tf]); grid on;
xlabel('t [s]'); ylabel('v [m/s]');
title('(c) velocidad de las cabinas');
legend('v_A', 'v_B', 'v^*', 'Location', 'best');

% (d) Corriente de armadura
subplot(2, 3, 4);
plot(r.t, r.ia, 'r'); hold on;
plot(ff.t, ff.i, 'k--', 'LineWidth', 1);
plot([0 tf], [0 0], 'k:');
xlim([0 tf]); grid on;
xlabel('t [s]'); ylabel('i_a [A]');
title('(d) corriente de armadura');
legend('i_a(t)', 'i_a^* requerida', 'Location', 'south');

% (e) Posicion de ambas cabinas
subplot(2, 3, 5);
plot(r.t, r.xA, 'b', r.t, r.xB, 'r', 'LineWidth', 1.2); hold on;
plot([0 tf], [p.xS p.xS], 'k:', [0 tf], [0 0], 'k:');
xlim([0 tf]); grid on;
xlabel('t [s]'); ylabel('x [m]');
title('(e) posicion de las cabinas');
legend('x_A', 'x_B', 'Location', 'east');

% (f) Tension de los cables (H6: positivas)
subplot(2, 3, 6);
plot(r.t, r.FA/1e3, 'b', r.t, r.FB/1e3, 'r'); hold on;
plot([0 tf], [0 0], 'k:');
xlim([0 tf]); grid on;
xlabel('t [s]'); ylabel('F [kN]');
title('(f) tension de los cables');
legend('F_A', 'F_B', 'Location', 'best');

exportar_figura_t6(fig, archivo);
end

function exportar_figura_t6(fig, nombre)
% EXPORTAR_FIGURA_T6  Guarda la figura en ../informe/figuras/ o en ./figuras/
aqui = fileparts(mfilename('fullpath'));
destino = fullfile(aqui, '..', 'informe', 'figuras');      % dentro del repositorio
if ~exist(destino, 'dir'), destino = fullfile(aqui, 'figuras'); end
if ~exist(destino, 'dir'), mkdir(destino); end
archivo = fullfile(destino, [nombre '.pdf']);
try
    exportgraphics(fig, archivo, 'ContentType', 'vector');
catch
    set(fig, 'PaperUnits', 'centimeters', 'PaperSize', [32 16.5], ...
        'PaperPosition', [0 0 32 16.5]);
    print(fig, archivo, '-dpdf');
end
fprintf('  figura: %s\n', archivo);
end
