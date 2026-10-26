function r = simular_t6(mdl, p, ff, tf, modo)
% =========================================================================
% SIMULAR_T6  Ejecuta un ensayo y devuelve las senales registradas.
%   r = simular_t6(mdl, p, ff, tf, 'simulink')  usa el modelo Simulink
%   r = simular_t6(mdl, p, ff, tf, 'ecuacion')  integra directamente la
%       ecuacion de estado (RK4, paso fijo p.MaxStep); sirve para verificar
%       que el diagrama Simulink implementa la misma ecuacion.
%   ff es la prealimentacion (prealimentacion_t6.m) y tf el tiempo final.
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
