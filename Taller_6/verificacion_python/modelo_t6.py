"""Verificacion independiente (Python) de la ecuacion de estado del Taller 6.

Mismas ecuaciones que ecuacion_estado_t6.m; integra con RK4 de paso fijo."""
import numpy as np, math

def params(**kw):
    p = dict(
        # fuente, convertidor y rectificador
        f=50.0, a=1.5, Rs=0.2, C=10e-3,
        # motor de CC (Taller 3)
        Ra=0.5, La=8e-3, Ka=2.6,
        # cadena rotacional (Taller 3)
        J1=0.35, b1=0.015, K1=2e-4, N=40.0, J2=3.0, b2=10.0, K2=2e-6, Jp=50.0, r=0.5,
        # carga: dos cabinas (Taller 2, sin roce seco)
        mA=1500.0, mB=1500.0, g=9.81, th=math.pi/6, gam=800.0,
        KS=1.5e5, KB=1.5e5, gS=29200.0, gB=29200.0, xS=100.0,
    )
    p.update(kw)
    p['Jeq'] = p['Jp'] + (p['mA'] + p['mB'])*p['r']**2
    p['Ke'] = p['KS'] + p['KB']; p['ge'] = p['gS'] + p['gB']
    p['w'] = 2*math.pi*p['f']; p['T'] = 1/p['f']
    return p

# ---------------- perfil S (quintico) -----------------
def s5(u):
    u = np.clip(u, 0, 1); return 6*u**5 - 15*u**4 + 10*u**3
def ds5(u):
    m = (u >= 0) & (u <= 1); return np.where(m, 30*u**2*(1-u)**2, 0.0)
def dds5(u):
    m = (u >= 0) & (u <= 1); return np.where(m, 60*u - 180*u**2 + 120*u**3, 0.0)

def profile(t, vc=2.0, Ta=8.0, Td=8.0, Tc=None, L=100.0, t0=0.0):
    if Tc is None:
        Tc = (L - vc*Ta/2 - vc*Td/2)/vc
    t = np.asarray(t) - t0
    u1 = t/Ta; u2 = (t - Ta - Tc)/Td
    v = vc*(s5(u1) - s5(u2))
    a = vc*(ds5(u1)/Ta - ds5(u2)/Td)
    j = vc*(dds5(u1)/Ta**2 - dds5(u2)/Td**2)
    v = np.where(t < 0, 0, v); a = np.where(t < 0, 0, a); j = np.where(t < 0, 0, j)
    return v, a, j, Ta + Tc + Td

# ------------- ecuacion de estado (igual a ecuacion_estado_t6.m) ---------------
def make_rhs(p, Vef_fun):
    a, Rs, C, Ra, La, Ka = p['a'], p['Rs'], p['C'], p['Ra'], p['La'], p['Ka']
    J1, b1, K1, N, J2, b2, K2, Jeq, r = p['J1'], p['b1'], p['K1'], p['N'], p['J2'], p['b2'], p['K2'], p['Jeq'], p['r']
    Fg = (p['mA'] - p['mB'])*p['g']*math.sin(p['th'])
    gam2 = 2*p['gam']; Ke, ge, xS = p['Ke'], p['ge'], p['xS']
    w, T = p['w'], p['T']
    s3h = math.sqrt(3)/2
    def Vs(t):
        return math.sqrt(2)*Vef_fun(t)*math.sin(w*t) if t >= 0 else 0.0
    def rhs(t, x):
        U, ia, wm, Mk1, w2, Mk2, wp, FkS, FkI, xA = x
        va_ = Vs(t); vb_ = Vs(t - T/4)
        v1 = a*va_; v2 = a*(-0.5*va_ + s3h*vb_); v3 = a*(-0.5*va_ - s3h*vb_)
        iD = 0.0
        if v1 > U: iD += (v1 - U)/Rs
        if v2 > U: iD += (v2 - U)/Rs
        if v3 > U: iD += (v3 - U)/Rs
        v = r*wp
        s2 = 1.0 if xA > xS else 0.0
        s3 = 1.0 if xA < 0 else 0.0
        Fam = s2*(FkS + ge*v) + s3*(FkI + ge*v)
        Fc = Fg + gam2*v + Fam
        return (
            (iD - ia)/C,
            (U - Ra*ia - Ka*wm)/La,
            (Ka*ia - b1*wm - Mk1)/J1,
            (wm - N*w2)/K1,
            (N*Mk1 - b2*w2 - Mk2)/J2,
            (w2 - wp)/K2,
            (Mk2 - r*Fc)/Jeq,
            Ke*s2*v,
            Ke*s3*v,
            v,
        ), iD
    return rhs, Vs

def simulate(p, Vef_fun, tf, dt=1e-4, x0=None, dec=10):
    rhs, Vs = make_rhs(p, Vef_fun)
    x = list(x0) if x0 is not None else [0.0]*10
    n = int(round(tf/dt))
    out_t = []; out_x = []; out_iD = []; out_Vs = []
    t = 0.0
    def add(a_, b_, h):
        return [ai + h*bi for ai, bi in zip(a_, b_)]
    for k in range(n + 1):
        if k % dec == 0:
            d, iD = rhs(t, x)
            out_t.append(t); out_x.append(list(x)); out_iD.append(iD); out_Vs.append(Vs(t))
        if k == n: break
        k1, _ = rhs(t, x)
        k2, _ = rhs(t + dt/2, add(x, k1, dt/2))
        k3, _ = rhs(t + dt/2, add(x, k2, dt/2))
        k4, _ = rhs(t + dt, add(x, k3, dt))
        x = [xi + dt/6*(a1 + 2*a2 + 2*a3 + a4) for xi, a1, a2, a3, a4 in zip(x, k1, k2, k3, k4)]
        t += dt
    return np.array(out_t), np.array(out_x), np.array(out_iD), np.array(out_Vs)
