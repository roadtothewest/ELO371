function v = convertidor_ac_ac(vs, vs_d, a)
% CONVERTIDOR_AC_AC  Convertidor ideal monofasico -> trifasico.
%   vs   : tension de entrada Vs(t), tomada como componente alfa
%   vs_d : Vs(t - T/4), la misma tension retrasada 90 grados (componente beta)
%   a    : razon de amplitud del convertidor
%   v    : tensiones de fase [va; vb; vc] (secuencia positiva, neutro comun)
% Es la transformada inversa de Clarke escalada por a. Este mismo codigo es
% el del bloque MATLAB Function "Convertidor_AC_AC" del modelo Simulink.
v = a*[vs; -0.5*vs + (sqrt(3)/2)*vs_d; -0.5*vs - (sqrt(3)/2)*vs_d];
end
