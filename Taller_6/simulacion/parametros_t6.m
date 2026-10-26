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
