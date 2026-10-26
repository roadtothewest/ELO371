import os, sys, time, math, numpy as np, pickle
AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
from modelo_t6 import *
from prealimentacion import feedforward_an
name, mA, mB = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
dt = float(sys.argv[4]) if len(sys.argv) > 4 else 1e-4
Ta, Td, vc, L = 8.0, 20.0, 2.0, 100.0
p = params(mA=mA, mB=mB)
tg = np.arange(0, 80, 0.005)
v, a, j, Ttrip = profile(tg, vc=vc, Ta=Ta, Td=Td, L=L)
ff = feedforward_an(p, tg, v, a, j)
Vef_fun = lambda t: float(np.interp(t, tg, ff['Vef']))
tf = Ttrip + 6
T, X, ID, VS = simulate(p, Vef_fun, tf, dt=dt, dec=max(1, int(round(5e-4/dt))))
os.makedirs(os.path.join(AQUI, "resultados"), exist_ok=True)
pickle.dump(dict(p=p, tg=tg, v=v, a=a, ff=ff, T=T, X=X, ID=ID, VS=VS, Ttrip=Ttrip, Ta=Ta, Td=Td), open(os.path.join(AQUI, 'resultados', f'fin_{name}.pkl'), 'wb'))
r = p['r']; vm = r*X[:, 6]; x = X[:, 9]
k = (T > 15) & (T < 40)
dv = np.gradient(vm, T)
s2 = x > p['xS']; s3 = x < 0
gs = p['g']*math.sin(p['th'])
FA = mA*(dv + gs) + p['gam']*vm + s2*(p['KS']/p['Ke']*X[:, 7] + p['gS']*vm) + s3*(p['KB']/p['Ke']*X[:, 8] + p['gB']*vm)
FB = mB*(-dv + gs) - p['gam']*vm + s2*(-p['KB']/p['Ke']*X[:, 7] - p['gB']*vm) + s3*(-p['KS']/p['Ke']*X[:, 8] - p['gS']*vm)
ka = np.argmax(x >= L) if (x >= L).any() else None
print(f"== {name}: mA={mA} mB={mB} Ttrip={Ttrip}")
print(f" ff cruise: U*={np.interp(30, tg, ff['U']):.2f} i*={np.interp(30, tg, ff['i']):.3f} Vm*={np.interp(30, tg, ff['Vm']):.2f} Vef*={np.interp(30, tg, ff['Vef']):.2f}; i* range [{ff['i'].min():.2f},{ff['i'].max():.2f}] ; Vef max {ff['Vef'].max():.1f}")
print(f" cruise v: mean {vm[k].mean():.5f} min {vm[k].min():.5f} max {vm[k].max():.5f}")
print(f" cruise U: mean {X[k,0].mean():.2f} min {X[k,0].min():.2f} max {X[k,0].max():.2f}")
print(f" cruise ia: mean {X[k,1].mean():.3f} min {X[k,1].min():.3f} max {X[k,1].max():.3f}")
print(f" cruise iD: mean {ID[k].mean():.3f} max {ID[k].max():.2f}")
print(f" cruise Mk1: mean {X[k,3].mean():.3f} min {X[k,3].min():.3f} max {X[k,3].max():.3f}; Mk2 mean {X[k,5].mean():.1f} min {X[k,5].min():.1f} max {X[k,5].max():.1f}")
print(f" global: max U {X[:,0].max():.1f}, ia [{X[:,1].min():.2f},{X[:,1].max():.2f}], iD max {ID.max():.1f}, Mk1 max {X[:,3].max():.1f}")
print(f" track err max {np.abs(vm-np.interp(T,tg,v)).max():.4f} ; in accel (t<{Ta}) {np.abs(vm-np.interp(T,tg,v))[T<Ta].max():.4f}")
print(f" arrival: v={vm[ka] if ka is not None else float('nan'):.4f} at t={T[ka] if ka is not None else float('nan'):.2f}; max x {x.max():.4f}; min x {x.min():.4f}; final x {x[-1]:.4f}, final v {vm[-1]:.5f}")
kk = T < Ttrip
print(f" cable tension: FA [{FA[kk].min():.0f},{FA[kk].max():.0f}] FB [{FB[kk].min():.0f},{FB[kk].max():.0f}]")
print(f" vmax {vm.max():.4f} amax {dv[kk].max():.3f} amin {dv[kk].min():.3f}")
