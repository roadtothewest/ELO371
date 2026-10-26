import os, sys, math, pickle, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from modelo_t6 import params
from prealimentacion import feedforward_an
OUT = os.path.join(AQUI, '..', 'informe', 'figuras', '')
plt.rcParams.update({'font.size': 7, 'axes.titlesize': 7.5, 'legend.fontsize': 6, 'lines.linewidth': 1.0,
                     'axes.grid': True, 'grid.alpha': 0.35, 'figure.dpi': 150})

def derived(d):
    p, T, X = d['p'], d['T'], d['X']
    r = p['r']; v = r*X[:, 6]; xA = X[:, 9]
    # aceleracion de la rueda desde la ecuacion de estado
    s2 = xA > p['xS']; s3 = xA < 0
    Fc = p['mA']*0 + (p['mA']-p['mB'])*p['g']*math.sin(p['th']) + 2*p['gam']*v + s2*(X[:, 7] + p['ge']*v) + s3*(X[:, 8] + p['ge']*v)
    acc = r*(X[:, 5] - r*Fc)/p['Jeq']
    gs = p['g']*math.sin(p['th'])
    FA = p['mA']*(acc + gs) + p['gam']*v + s2*(p['KS']/p['Ke']*X[:, 7] + p['gS']*v) + s3*(p['KB']/p['Ke']*X[:, 8] + p['gB']*v)
    FB = p['mB']*(-acc + gs) - p['gam']*v - s2*(p['KB']/p['Ke']*X[:, 7] + p['gB']*v) - s3*(p['KS']/p['Ke']*X[:, 8] + p['gS']*v)
    return v, xA, FA, FB, acc

def phases(p, d, t):
    Vef = np.interp(t, d['tg'], d['ff']['Vef']); Vefd = np.interp(t - p['T']/4, d['tg'], d['ff']['Vef'])
    va = math.sqrt(2)*Vef*np.sin(p['w']*t); vb_ = math.sqrt(2)*Vefd*np.sin(p['w']*(t - p['T']/4))
    s3h = math.sqrt(3)/2
    return p['a']*va, p['a']*(-0.5*va + s3h*vb_), p['a']*(-0.5*va - s3h*vb_)

def movmean(x, n):
    k = np.ones(n)/n
    y = np.convolve(x, k, mode='same')
    h = n//2
    y[:h] = x[:h]; y[-h:] = x[-h:]
    return y

def fig_ensayo(name, out):
    d = pickle.load(open(os.path.join(AQUI, 'resultados', f'fin_{name}.pkl'), 'rb'))
    p, T, X, tg, ff = d['p'], d['T'], d['X'], d['tg'], d['ff']
    v, xA, FA, FB, acc = derived(d)
    tf = T[-1]
    fig, ax = plt.subplots(2, 3, figsize=(7.2, 4.1))
    a = ax[0, 0]
    dtl = T[1] - T[0]; Np = int(round(1/(3*p['f'])/dtl))
    a.plot(T, X[:, 0], color=(0.65, 0.78, 0.95), lw=0.6, label='$U(t)$')
    a.plot(T, movmean(X[:, 0], Np), 'b', lw=1.1, label='promedio')
    a.plot(tg, ff['U'], 'k--', lw=0.9, label='$U^*$ requerido')
    a.set_xlim(0, tf); a.set_xlabel('t [s]'); a.set_ylabel('U [V]'); a.set_title('(a) voltaje en el motor')
    a.legend(loc='lower center')
    a = ax[0, 1]
    t0 = round(d['Ta'] + (d['Ttrip'] - d['Ta'] - d['Td'])/2)
    k = (T >= t0) & (T <= t0 + 3/p['f'])
    tt = T[k]; ph = phases(p, d, tt)
    for q in ph: a.plot((tt - t0)*1e3, q, color='0.6', lw=0.7)
    a.plot((tt - t0)*1e3, np.max(np.vstack(ph), axis=0), 'k--', lw=0.7)
    a.plot((tt - t0)*1e3, X[k, 0], 'b', lw=1.4)
    yl = 1.1*np.abs(ph[0]).max(); a.set_ylim(-yl, yl); a.set_xlim(0, 3e3/p['f'])
    a.set_xlabel('$t - t_0$ [ms]'); a.set_ylabel('[V]'); a.set_title(f'(b) detalle en crucero, $t_0$ = {t0:.0f} s')
    a = ax[0, 2]
    a.plot(T, v, 'b', lw=1.1, label='$v_A$'); a.plot(T, -v, 'r', lw=1.1, label='$v_B$')
    a.plot(tg, d['v'], 'k--', lw=0.8, label='$v^*$'); a.plot(tg, -d['v'], 'k--', lw=0.8)
    a.set_xlim(0, tf); a.set_xlabel('t [s]'); a.set_ylabel('v [m/s]'); a.set_title('(c) velocidad de las cabinas'); a.legend(loc='center')
    a = ax[1, 0]
    a.plot(T, X[:, 1], 'r', lw=0.8, label='$i_a(t)$'); a.plot(tg, ff['i'], 'k--', lw=0.9, label='$i_a^*$ requerida')
    a.axhline(0, color='k', ls=':', lw=0.8)
    a.set_xlim(0, tf); a.set_xlabel('t [s]'); a.set_ylabel('$i_a$ [A]'); a.set_title('(d) corriente de armadura'); a.legend(loc='lower center')
    a = ax[1, 1]
    a.plot(T, xA, 'b', lw=1.1, label='$x_A$'); a.plot(T, p['xS'] - xA, 'r', lw=1.1, label='$x_B$')
    a.axhline(p['xS'], color='k', ls=':', lw=0.8); a.axhline(0, color='k', ls=':', lw=0.8)
    a.set_xlim(0, tf); a.set_xlabel('t [s]'); a.set_ylabel('x [m]'); a.set_title('(e) posición de las cabinas'); a.legend(loc='center right')
    a = ax[1, 2]
    a.plot(T, FA/1e3, 'b', lw=0.9, label='$F_A$'); a.plot(T, FB/1e3, 'r', lw=0.9, label='$F_B$')
    a.axhline(0, color='k', ls=':', lw=0.8)
    a.set_xlim(0, tf); a.set_xlabel('t [s]'); a.set_ylabel('F [kN]'); a.set_title('(f) tensión de los cables'); a.legend(loc='best')
    if name == 'dn':
        a.set_ylim(-10, 20)
    fig.tight_layout(pad=0.4, w_pad=0.6, h_pad=0.6)
    fig.savefig(OUT + out + '.pdf')
    plt.close(fig)

for name, out in [('eq', 'fig_ensayo1'), ('up', 'fig_ensayo2a'), ('dn', 'fig_ensayo2b')]:
    fig_ensayo(name, out)

# ---------------- regimen permanente ----------------
p = params()
vc = 2.0
cm = p['b1'] + (p['b2'] + 2*p['gam']*p['r']**2)/p['N']**2
A = p['Ka']*p['N']/p['r'] + p['Ra']*cm*p['N']/(p['Ka']*p['r'])
B = p['Ra']*p['r']*p['g']*math.sin(p['th'])/(p['Ka']*p['N'])
dm = np.linspace(-900, 900, 361)
i_s = (cm*p['N']/p['r']*vc + p['r']*dm*p['g']*math.sin(p['th'])/p['N'])/p['Ka']
U_s = A*vc + B*dm
from scipy.optimize import brentq
def vm_of(U, I):
    if I <= 0: return U
    rhs = math.pi*p['Rs']*I/(3*U)
    th = brentq(lambda x: math.tan(x) - x - rhs, 1e-12, math.pi/2 - 1e-9)
    return U/math.cos(th)
Vs_s = np.array([vm_of(u, i) for u, i in zip(U_s, i_s)])/(p['a']*math.sqrt(2))
dm_lim = -cm*p['N']**2*vc/(p['r']**2*p['g']*math.sin(p['th']))
print("A", A, "B", B, "dm_lim", dm_lim, "U(0)", A*vc, "i(0)", np.interp(0, dm, i_s), "Vs(0)", np.interp(0, dm, Vs_s))
for dmv in (600, -600):
    print(dmv, "U", A*vc + B*dmv, "i", np.interp(dmv, dm, i_s), "Vs", np.interp(dmv, dm, Vs_s))
sim = {}
for name, dmv in [('eq', 0), ('up', 600), ('dn', -600)]:
    d = pickle.load(open(os.path.join(AQUI, 'resultados', f'fin_{name}.pkl'), 'rb'))
    k = (d['T'] > 12) & (d['T'] < 40)
    sim[name] = (dmv, d['X'][k, 0].mean(), d['X'][k, 1].mean())
fig, ax = plt.subplots(1, 3, figsize=(7.2, 2.15))
a = ax[0]; a.plot(dm, U_s, 'b', lw=1.4)
for s in sim.values(): a.plot(s[0], s[1], 'ko', mfc='y', ms=4)
a.set_xlabel(r'$\Delta m = m_A - m_B$ [kg]'); a.set_ylabel('U [V]'); a.set_title('(a) voltaje en el motor, $v$ = 2 m/s')
a = ax[1]; a.plot(dm, i_s, 'r', lw=1.4)
for s in sim.values(): a.plot(s[0], s[2], 'ko', mfc='y', ms=4)
a.axhline(0, color='k', ls=':', lw=0.8); a.axvline(dm_lim, color='k', ls='--', lw=0.8)
a.axvspan(-900, dm_lim, color='0.85')
a.text(-885, 36, 'requiere\nfrenar', ha='left', va='top', fontsize=6)
a.set_xlim(-900, 900)
a.set_xlabel(r'$\Delta m = m_A - m_B$ [kg]'); a.set_ylabel('$i_a$ [A]'); a.set_title('(b) corriente de armadura')
a = ax[2]; a.plot(dm, Vs_s, 'k', lw=1.4); a.axhline(p['Vn'] if 'Vn' in p else 220, color='k', ls=':', lw=0.8)
a.set_xlabel(r'$\Delta m = m_A - m_B$ [kg]'); a.set_ylabel('$V_s$ [V rms]'); a.set_title('(c) tensión monofásica requerida')
fig.tight_layout(pad=0.4, w_pad=0.8)
fig.savefig(OUT + 'fig_regimen.pdf')
