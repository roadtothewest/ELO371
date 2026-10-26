function construir_modelo_t6(mdl)
% =========================================================================
% CONSTRUIR_MODELO_T6  Construye el modelo Simulink del Taller 6.
%   construir_modelo_t6('ascensor_duple_t6') crea y guarda el .slx.
%
%   El diagrama es consecuencia directa de la ecuacion de estado (punto 12
%   del informe): cada fila implementa una ecuacion con la cadena
%   From -> Gain -> Sum -> Gain(1/coef) -> Integrator -> Goto, igual que en
%   el Taller 3. La fuente, el convertidor AC-AC y el rectificador van en la
%   parte superior; el convertidor y el rectificador son bloques MATLAB
%   Function cuyo codigo va escrito al final de este archivo (es el mismo de
%   las funciones convertidor_ac_ac y rectificador_m3 de tests.m).
%   Todos los parametros se leen de la estructura p (parametros_t6 en tests.m).
%   Solo se ejecuta una vez: despues basta el .slx y tests.m.
% =========================================================================
if nargin < 1, mdl = 'ascensor_duple_t6'; end
aqui = fileparts(mfilename('fullpath'));
if bdIsLoaded(mdl), close_system(mdl, 0); end
archivo = fullfile(aqui, [mdl '.slx']);
if exist(archivo, 'file'), delete(archivo); end
new_system(mdl);

%% ---------------------------------------------------- FUENTE Y CONVERTIDOR
add_block('simulink/Sources/From Workspace', [mdl '/Vef'], ...
    'VariableName', 'Vef_ts', 'Interpolate', 'on', ...
    'OutputAfterFinalValue', 'Holding final value', 'Position', [30 30 110 60]);
add_block('simulink/Sources/Sine Wave', [mdl '/Portadora'], ...
    'Amplitude', 'sqrt(2)', 'Frequency', '2*pi*p.f', 'Phase', '0', ...
    'SampleTime', '0', 'Position', [40 90 80 120]);
add_block('simulink/Math Operations/Product', [mdl '/Fuente_Vs'], ...
    'Position', [160 50 190 100]);
add_line(mdl, 'Vef/1', 'Fuente_Vs/1', 'autorouting', 'on');
add_line(mdl, 'Portadora/1', 'Fuente_Vs/2', 'autorouting', 'on');
goto_(mdl, 'Vs', 'Fuente_Vs/1', [230 65 290 85]);

% Convertidor AC-AC: Vs (alfa), Vs retrasado T/4 (beta) y razon a
from_(mdl, 'Conv_Vs', 'Vs', [30 160 80 180]);
from_(mdl, 'Conv_Vs_d', 'Vs', [30 200 80 220]);
add_block('simulink/Continuous/Transport Delay', [mdl '/Retardo_T4'], ...
    'DelayTime', '1/(4*p.f)', 'InitialOutput', '0', 'BufferSize', '4096', ...
    'Position', [120 195 170 225]);
add_line(mdl, 'Conv_Vs_d/1', 'Retardo_T4/1', 'autorouting', 'on');
add_block('simulink/Sources/Constant', [mdl '/Const_a'], 'Value', 'p.a', ...
    'Position', [120 240 170 260]);
mfun_(mdl, 'Convertidor_AC_AC', codigo_convertidor(), [250 160 380 260]);
add_line(mdl, 'Conv_Vs/1', 'Convertidor_AC_AC/1', 'autorouting', 'on');
add_line(mdl, 'Retardo_T4/1', 'Convertidor_AC_AC/2', 'autorouting', 'on');
add_line(mdl, 'Const_a/1', 'Convertidor_AC_AC/3', 'autorouting', 'on');
goto_(mdl, 'vabc', 'Convertidor_AC_AC/1', [430 200 490 220]);

% Rectificador de 3 diodos: tensiones de fase, tension del bus U y Rs
from_(mdl, 'Rect_vabc', 'vabc', [30 300 80 320]);
from_(mdl, 'Rect_U', 'U', [30 335 80 355]);
add_block('simulink/Sources/Constant', [mdl '/Const_Rs'], 'Value', 'p.Rs', ...
    'Position', [120 370 170 390]);
mfun_(mdl, 'Rectificador', codigo_rectificador(), [250 295 380 395]);
add_line(mdl, 'Rect_vabc/1', 'Rectificador/1', 'autorouting', 'on');
add_line(mdl, 'Rect_U/1', 'Rectificador/2', 'autorouting', 'on');
add_line(mdl, 'Const_Rs/1', 'Rectificador/3', 'autorouting', 'on');
goto_(mdl, 'iD', 'Rectificador/1', [430 335 490 355]);

%% ---------------------------------------------------- ECUACIONES DE ESTADO
% fila_(mdl, estado, {origen, ganancia}, signos, 1/coef, condicion inicial, y)
y = 450; dy = 110;
fila_(mdl, 'U',   {'iD', ''; 'ia', ''},                         '+-',  '1/p.C',   'p.x0(1)', y); y = y + dy;
fila_(mdl, 'ia',  {'U', ''; 'ia', 'p.Ra'; 'wm', 'p.Ka'},        '+--', '1/p.La',  'p.x0(2)', y); y = y + dy;
fila_(mdl, 'wm',  {'ia', 'p.Ka'; 'wm', 'p.b1'; 'Mk1', ''},      '+--', '1/p.J1',  'p.x0(3)', y); y = y + dy;
fila_(mdl, 'Mk1', {'wm', ''; 'w2', 'p.N'},                      '+-',  '1/p.K1',  'p.x0(4)', y); y = y + dy;
fila_(mdl, 'w2',  {'Mk1', 'p.N'; 'w2', 'p.b2'; 'Mk2', ''},      '+--', '1/p.J2',  'p.x0(5)', y); y = y + dy;
fila_(mdl, 'Mk2', {'w2', ''; 'wp', ''},                         '+-',  '1/p.K2',  'p.x0(6)', y); y = y + dy;
fila_(mdl, 'wp',  {'Mk2', ''; 'Fc', 'p.r'},                     '+-',  '1/p.Jeq', 'p.x0(7)', y);
goto_(mdl, 'dwp', 'wp_inv/1', [420 y+70 480 y+90]);             % aceleracion de la rueda
y = y + dy;

% Velocidad de la cabina A (salida y_1)
from_(mdl, 'v_de_wp', 'wp', [30 y 80 y+20]);
add_block('simulink/Math Operations/Gain', [mdl '/v_r'], 'Gain', 'p.r', ...
    'Position', [130 y 190 y+20]);
add_line(mdl, 'v_de_wp/1', 'v_r/1', 'autorouting', 'on');
goto_(mdl, 'v', 'v_r/1', [580 y 640 y+20]);
y = y + dy;

% Resortes de los topes: s2 (xA > xS) y s3 (xA < 0) gobernados por la posicion
fila_tope_(mdl, 'FkS', 'u2 > Threshold',  'p.xS', true,  'p.x0(8)', y); y = y + dy;
fila_tope_(mdl, 'FkI', 'u2 >= Threshold', '0',    false, 'p.x0(9)', y); y = y + dy;

% Posicion de la cabina A
from_(mdl, 'xA_de_v', 'v', [30 y 80 y+20]);
add_block('simulink/Continuous/Integrator', [mdl '/xA_int'], ...
    'InitialCondition', 'p.x0(10)', 'Position', [460 y-5 495 y+25]);
add_line(mdl, 'xA_de_v/1', 'xA_int/1', 'autorouting', 'on');
goto_(mdl, 'xA', 'xA_int/1', [580 y 640 y+20]);

%% ---------------------------------------------------- FUERZA DE LA CARGA Fc
% Fc = Fg + 2*gamma*v + s2*(FkS + ge*v) + s3*(FkI + ge*v)
x0 = 760; yc = 450;
add_block('simulink/Sources/Constant', [mdl '/Fc_Fg'], 'Value', 'p.Fg', ...
    'Position', [x0+150 yc x0+200 yc+20]);
from_(mdl, 'Fc_v1', 'v', [x0 yc+50 x0+50 yc+70]);
add_block('simulink/Math Operations/Gain', [mdl '/Fc_roce'], 'Gain', '2*p.gamma', ...
    'Position', [x0+130 yc+50 x0+200 yc+70]);
add_line(mdl, 'Fc_v1/1', 'Fc_roce/1', 'autorouting', 'on');
amortiguador_(mdl, 'S', 'FkS', 'u2 > Threshold',  'p.xS', true,  x0, yc+110);
amortiguador_(mdl, 'I', 'FkI', 'u2 >= Threshold', '0',    false, x0, yc+250);
add_block('simulink/Math Operations/Sum', [mdl '/Fc_suma'], 'Inputs', '++++', ...
    'IconShape', 'rectangular', 'Position', [x0+380 yc x0+405 yc+320]);
add_line(mdl, 'Fc_Fg/1', 'Fc_suma/1', 'autorouting', 'on');
add_line(mdl, 'Fc_roce/1', 'Fc_suma/2', 'autorouting', 'on');
add_line(mdl, 'Fc_swS/1', 'Fc_suma/3', 'autorouting', 'on');
add_line(mdl, 'Fc_swI/1', 'Fc_suma/4', 'autorouting', 'on');
goto_(mdl, 'Fc', 'Fc_suma/1', [x0+450 yc+150 x0+510 yc+170]);

%% ---------------------------------------------------- REGISTRO DE SENALES
senales = {'Vs', 'vabc', 'U', 'iD', 'ia', 'wm', 'Mk1', 'Mk2', 'v', 'dwp', ...
           'xA', 'FkS', 'FkI', 'Fc'};
xr = 1400; yr = 30;
for k = 1:numel(senales)
    s = senales{k};
    from_(mdl, ['log_' s], s, [xr yr xr+50 yr+20]);
    add_block('simulink/Sinks/To Workspace', [mdl '/out_' s], ...
        'VariableName', s, 'SaveFormat', 'Timeseries', 'SampleTime', 'p.Ts_log', ...
        'Position', [xr+100 yr-2 xr+180 yr+22]);
    add_line(mdl, ['log_' s '/1'], ['out_' s '/1'], 'autorouting', 'on');
    yr = yr + 45;
end

%% ---------------------------------------------------- CONFIGURACION
% Sistema rigido (modos de 1 ms a 60 s): solver implicito de paso variable
set_param(mdl, 'SolverType', 'Variable-step', 'Solver', 'ode23t', ...
    'MaxStep', '1e-4', 'RelTol', '1e-4', 'AbsTol', 'auto', 'StopTime', '70');
save_system(mdl, archivo);
fprintf('Modelo %s guardado en %s\n', mdl, archivo);

% Figura del diagrama para el informe (carpeta del informe si existe)
destino = fullfile(aqui, '..', 'informe', 'figuras');
if ~exist(destino, 'dir'), destino = aqui; end
try
    open_system(mdl);
    print(['-s' mdl], '-dpdf', fullfile(destino, 'diagrama_simulink.pdf'));
catch err
    fprintf('No se pudo exportar el diagrama: %s\n', err.message);
end
end

% =========================================================================
function fila_(mdl, tag, ents, signos, ginv, ci, y)
% Fila de una ecuacion de estado: d(tag)/dt = ginv * sum(signo_k * g_k * origen_k)
n  = size(ents, 1);
hS = 30*n;
sumN = [tag '_suma'];
add_block('simulink/Math Operations/Sum', [mdl '/' sumN], 'Inputs', signos, ...
    'IconShape', 'rectangular', 'Position', [260 y 285 y+hS]);
for k = 1:n
    yk = y + 30*(k-1) + 5;
    fN = sprintf('%s_de_%s_%d', tag, ents{k, 1}, k);
    from_(mdl, fN, ents{k, 1}, [30 yk 80 yk+20]);
    if isempty(ents{k, 2})
        add_line(mdl, [fN '/1'], sprintf('%s/%d', sumN, k), 'autorouting', 'on');
    else
        gN = sprintf('%s_g%d', tag, k);
        add_block('simulink/Math Operations/Gain', [mdl '/' gN], 'Gain', ents{k, 2}, ...
            'Position', [130 yk 200 yk+20]);
        add_line(mdl, [fN '/1'], [gN '/1'], 'autorouting', 'on');
        add_line(mdl, [gN '/1'], sprintf('%s/%d', sumN, k), 'autorouting', 'on');
    end
end
yc = y + hS/2;
add_block('simulink/Math Operations/Gain', [mdl '/' tag '_inv'], 'Gain', ginv, ...
    'Position', [330 yc-15 400 yc+15]);
add_block('simulink/Continuous/Integrator', [mdl '/' tag '_int'], ...
    'InitialCondition', ci, 'Position', [460 yc-15 495 yc+15]);
add_line(mdl, [sumN '/1'], [tag '_inv/1'], 'autorouting', 'on');
add_line(mdl, [tag '_inv/1'], [tag '_int/1'], 'autorouting', 'on');
goto_(mdl, tag, [tag '_int/1'], [580 yc-10 640 yc+10]);
end

function fila_tope_(mdl, tag, criterio, umbral, superior, ci, y)
% d(tag)/dt = Ke * s(xA) * v ; el Switch deja pasar v solo durante el contacto
from_(mdl, [tag '_v'], 'v', [30 y 80 y+20]);
from_(mdl, [tag '_x'], 'xA', [30 y+30 80 y+50]);
add_block('simulink/Sources/Constant', [mdl '/' tag '_cero'], 'Value', '0', ...
    'Position', [30 y+60 80 y+80]);
add_block('simulink/Signal Routing/Switch', [mdl '/' tag '_sw'], ...
    'Criteria', criterio, 'Threshold', umbral, 'Position', [200 y 240 y+80]);
if superior   % s2: pasa v si xA > xS
    add_line(mdl, [tag '_v/1'], [tag '_sw/1'], 'autorouting', 'on');
    add_line(mdl, [tag '_cero/1'], [tag '_sw/3'], 'autorouting', 'on');
else          % s3: pasa v si xA < 0 (entradas intercambiadas)
    add_line(mdl, [tag '_cero/1'], [tag '_sw/1'], 'autorouting', 'on');
    add_line(mdl, [tag '_v/1'], [tag '_sw/3'], 'autorouting', 'on');
end
add_line(mdl, [tag '_x/1'], [tag '_sw/2'], 'autorouting', 'on');
add_block('simulink/Math Operations/Gain', [mdl '/' tag '_inv'], 'Gain', 'p.Ke', ...
    'Position', [330 y+25 400 y+55]);
add_block('simulink/Continuous/Integrator', [mdl '/' tag '_int'], ...
    'InitialCondition', ci, 'Position', [460 y+25 495 y+55]);
add_line(mdl, [tag '_sw/1'], [tag '_inv/1'], 'autorouting', 'on');
add_line(mdl, [tag '_inv/1'], [tag '_int/1'], 'autorouting', 'on');
goto_(mdl, tag, [tag '_int/1'], [580 y+30 640 y+50]);
end

function amortiguador_(mdl, sufijo, tagF, criterio, umbral, superior, x0, y)
% Fuerza de los dos amortiguadores en contacto: s*(Fk + ge*v)
b = ['Fc_' sufijo];
from_(mdl, [b '_F'], tagF, [x0 y x0+50 y+20]);
from_(mdl, [b '_v'], 'v', [x0 y+30 x0+50 y+50]);
add_block('simulink/Math Operations/Gain', [mdl '/' b '_ge'], 'Gain', 'p.ge', ...
    'Position', [x0+80 y+30 x0+130 y+50]);
add_block('simulink/Math Operations/Sum', [mdl '/' b '_suma'], 'Inputs', '++', ...
    'IconShape', 'rectangular', 'Position', [x0+160 y x0+180 y+50]);
add_line(mdl, [b '_F/1'], [b '_suma/1'], 'autorouting', 'on');
add_line(mdl, [b '_v/1'], [b '_ge/1'], 'autorouting', 'on');
add_line(mdl, [b '_ge/1'], [b '_suma/2'], 'autorouting', 'on');
from_(mdl, [b '_x'], 'xA', [x0 y+70 x0+50 y+90]);
add_block('simulink/Sources/Constant', [mdl '/' b '_cero'], 'Value', '0', ...
    'Position', [x0 y+100 x0+50 y+120]);
add_block('simulink/Signal Routing/Switch', [mdl '/Fc_sw' sufijo], ...
    'Criteria', criterio, 'Threshold', umbral, 'Position', [x0+240 y x0+280 y+120]);
if superior
    add_line(mdl, [b '_suma/1'], ['Fc_sw' sufijo '/1'], 'autorouting', 'on');
    add_line(mdl, [b '_cero/1'], ['Fc_sw' sufijo '/3'], 'autorouting', 'on');
else
    add_line(mdl, [b '_cero/1'], ['Fc_sw' sufijo '/1'], 'autorouting', 'on');
    add_line(mdl, [b '_suma/1'], ['Fc_sw' sufijo '/3'], 'autorouting', 'on');
end
add_line(mdl, [b '_x/1'], ['Fc_sw' sufijo '/2'], 'autorouting', 'on');
end

function from_(mdl, nombre, tag, pos)
add_block('simulink/Signal Routing/From', [mdl '/' nombre], 'GotoTag', tag, ...
    'Position', pos);
end

function goto_(mdl, tag, origen, pos)
add_block('simulink/Signal Routing/Goto', [mdl '/goto_' tag], 'GotoTag', tag, ...
    'Position', pos);
add_line(mdl, origen, ['goto_' tag '/1'], 'autorouting', 'on');
end

function mfun_(mdl, nombre, codigo, pos)
% Bloque MATLAB Function con el codigo indicado
blk = [mdl '/' nombre];
add_block('simulink/User-Defined Functions/MATLAB Function', blk, 'Position', pos);
try
    cfg = get_param(blk, 'MATLABFunctionConfiguration');
    cfg.FunctionScript = codigo;
catch
    ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
    ch.Script = codigo;
end
end

function c = codigo_convertidor()
% Convertidor ideal monofasico -> trifasico (H1): inversa de Clarke de
% alfa = Vs(t) y beta = Vs(t - T/4), escalada por la razon a.
c = strjoin({
    'function v = convertidor_ac_ac(vs, vs_d, a)'
    '% Convertidor ideal 1F -> 3F: vs = alfa, vs_d = Vs(t - T/4) = beta'
    'v = a*[vs; -0.5*vs + (sqrt(3)/2)*vs_d; -0.5*vs - (sqrt(3)/2)*vs_d];'
    'end'}, newline);
end

function c = codigo_rectificador()
% Rectificador de 3 diodos de catodo comun, cada uno en serie con Rs (H2).
c = strjoin({
    'function iD = rectificador_m3(v, U, Rs)'
    '% Corriente total de los diodos que conducen hacia el bus de tension U'
    'iD = 0;'
    'for k = 1:3'
    '    if v(k) > U'
    '        iD = iD + (v(k) - U)/Rs;'
    '    end'
    'end'
    'end'}, newline);
end
