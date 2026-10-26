import math, numpy as np
from scipy.optimize import brentq
def feedforward_an(p, t, v, a, j):
    r, N, Ka, Ra, La = p['r'], p['N'], p['Ka'], p['Ra'], p['La']
    Jtot = p['J1'] + (p['J2'] + p['Jeq'])/N**2
    cm = p['b1'] + (p['b2'] + 2*p['gam']*r**2)/N**2
    Mg = r*(p['mA'] - p['mB'])*p['g']*math.sin(p['th'])/N
    wm = N*v/r
    Mm = Jtot*N/r*a + cm*N/r*v + Mg
    dMm = Jtot*N/r*j + cm*N/r*a
    i = Mm/Ka; di = dMm/Ka
    U = Ra*i + La*di + Ka*wm
    dU = np.gradient(U, t)
    ID = i + p['C']*dU
    Vm = np.empty_like(U)
    for k in range(len(U)):
        if ID[k] <= 0 or U[k] <= 1e-9:
            Vm[k] = max(U[k], 0.0)
        else:
            rhs = math.pi*p['Rs']*ID[k]/(3*U[k])
            th = brentq(lambda x: math.tan(x) - x - rhs, 1e-12, math.pi/2 - 1e-9)
            Vm[k] = U[k]/math.cos(th)
    return dict(U=U, i=i, wm=wm, Mm=Mm, ID=ID, Vm=Vm, Vef=Vm/(p['a']*math.sqrt(2)), Jtot=Jtot, cm=cm, Mg=Mg)
