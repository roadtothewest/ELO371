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
