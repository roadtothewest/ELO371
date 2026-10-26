function [dx, iD, dwp] = ecuacion_estado_t6(x, vabc, p)
% =========================================================================
% ECUACION_ESTADO_T6  Ecuacion de estado del Taller 6, dx/dt = f(x, u).
%   x    = [U; ia; wm; Mk1; w2; Mk2; wp; FkS; FkI; xA]  (punto 11 del informe)
%   vabc = tensiones de fase del convertidor AC-AC (entrada del rectificador)
%   Devuelve tambien la corriente de diodos iD y la aceleracion de la rueda.
% Es la misma ecuacion que implementa fila a fila el modelo Simulink; se usa
% en tests.m para verificar el diagrama por integracion directa.
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
