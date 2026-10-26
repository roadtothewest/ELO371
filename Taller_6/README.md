# Taller 6 – Sistema eléctrico – rotacional – traslación

Ascensor duple de desplazamiento diagonal: `Vs(t)` monofásico → convertidor AC–AC (1φ→3φ)
→ rectificador trifásico de 3 diodos con condensador → motor CC → ejes del Taller 3 → rueda
→ dos cabinas contrapesadas del Taller 2 (sin roce seco).

## Estructura

```
Taller_6/
  informe/
    main.tex                        informe (estructura de los Talleres 2 y 3)
    Taller_6_Grupo_3_ELO371.pdf     informe compilado
    circuito/circuito_equivalente.tex   diagrama eléctrico equivalente (pregunta 5)
    figuras/                        figuras que incluye el informe
  simulacion/
    tests.m                 script principal: ejecuta los ensayos (preguntas 3 y 4)
    parametros_t6.m         valores asignados (pregunta 2)
    construir_modelo_t6.m   genera ascensor_duple_t6.slx desde la ecuación de estado
    convertidor_ac_ac.m     código del bloque MATLAB Function del convertidor
    rectificador_m3.m       código del bloque MATLAB Function del rectificador
    rectificador_promedio.m modelo promedio (estático) del rectificador
    prealimentacion_t6.m    excitación Vs(t) por dinámica inversa (modelo promedio)
    ecuacion_estado_t6.m    ecuación de estado dx/dt = f(x,u) (verificación)
    simular_t6.m            corre un ensayo en Simulink o integrando la ecuación
    graficar_ensayo_t6.m, exportar_figura_t6.m
  verificacion_python/      integración independiente de la misma ecuación de estado
```

## Ejecutar (MATLAB)

```matlab
cd Taller_6/simulacion
tests
```

`tests.m` construye `ascensor_duple_t6.slx` si no existe (y exporta
`informe/figuras/diagrama_simulink.pdf`, que el informe incluye si está presente),
corre los ensayos 1, 2a y 2b sobre el modelo Simulink y exporta las figuras a
`informe/figuras/`. Con `VERIFICAR = true` integra además la ecuación de estado con
RK4 y muestra la diferencia máxima entre ambas soluciones, que debe ser pequeña.

**Importante:** las figuras que trae hoy el informe se obtuvieron integrando la ecuación de
estado (`verificacion_python/`, idéntica a `ecuacion_estado_t6.m`; también se probó el
modo `'ecuacion'` de `tests.m` en Octave), porque el modelo Simulink no se pudo ejecutar
en el entorno donde se preparó. Al correr `tests.m` en MATLAB se reemplazan por las del
modelo Simulink; luego hay que recompilar el informe.

## Compilar el informe

```bash
cd Taller_6/informe/circuito && pdflatex circuito_equivalente.tex && cp circuito_equivalente.pdf ../figuras/
cd .. && latexmk -pdf main.tex && cp main.pdf Taller_6_Grupo_3_ELO371.pdf
```

## Entrega

```matlab
cd Taller_6
zip('Taller_6_Grupo_3_ELO371.zip', {'informe/Taller_6_Grupo_3_ELO371.pdf', 'simulacion/*.m', 'simulacion/*.slx'});
```
