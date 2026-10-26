function iD = rectificador_m3(v, U, Rs)
% RECTIFICADOR_M3  Rectificador trifasico de media onda (3 diodos, catodo comun).
%   v  : tensiones de fase [va; vb; vc] respecto del neutro
%   U  : tension del condensador del bus de CC (estado)
%   Rs : resistencia serie de cada fase
%   iD : corriente total entregada por los diodos al bus
% Cada diodo es ideal en serie con Rs: conduce solo si su fase supera a U.
% Este mismo codigo es el del bloque MATLAB Function "Rectificador".
iD = 0;
for k = 1:3
    if v(k) > U
        iD = iD + (v(k) - U)/Rs;
    end
end
end
