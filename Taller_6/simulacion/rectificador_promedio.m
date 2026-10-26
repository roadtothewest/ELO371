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
