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
