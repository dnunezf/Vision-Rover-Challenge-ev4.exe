# Mediciones

Todo lo que acá dice "medido" se midió de verdad. Lo que es estimación, lo dice.
Los resultados negativos están anotados a propósito: sin ellos, dentro de dos
semanas alguien vuelve a intentar lo mismo.

---

# 1. Robot — comportamiento físico

Prueba p5, 3-oct-2026. Las dos placas dieron lo mismo en las tres mediciones,
así que el `config.py` de los dos lleva los mismos valores.

| Qué | Valor | Cómo se midió |
|---|---|---|
| Zona muerta, ruedas en el aire | 0.08 | subiendo el throttle de a poco hasta que giran |
| **Zona muerta, apoyado en el piso** | **0.10** | lo mismo, con el peso encima |
| Desvío al avanzar recto | 0.5 cm en 30 cm | cinta métrica |
| Velocidad a throttle 0.45 | 27-30 cm en 5 s = **2.7 celdas/s** | cronómetro |
| Giro de 90° a throttle 0.30 | **0.8 s** | cronómetro |

**Vale la del piso, no la del aire.** En el aire las ruedas no cargan el peso
del robot, así que arrancan antes. El número que importa para el control es el
que tiene el robot apoyado: 0.10.

**El desvío no se compensa.** 0.5 cm en 30 cm es menos de un grado, dentro del
error de medir con cinta. Compensar un desvío que no existe lo crea.
`COMPENSACION_DESVIO = 0.0`.

## El convenio de signos de los motores

Determinado con una prueba de cuatro combinaciones y un video, porque el
ejemplo de CENFOTEC **no sirve** para estos robots:

```python
# codigos/test_motores.py, de CENFOTEC — en NUESTROS robots esto GIRA
ib.motor_1.throttle = speed
ib.motor_2.throttle = speed
```

Los motores están montados espejados. El convenio correcto, verificado con la
p7 el 3-oct:

| Pedido | motor_1 | motor_2 |
|---|---|---|
| ADELANTE  (v=+0.45) | −0.505 | +0.505 |
| ATRÁS     (v=−0.45) | +0.505 | −0.505 |
| GIRA IZQ  (w=+0.30) | +0.370 | +0.370 |
| GIRA DER  (w=−0.30) | −0.370 | −0.370 |

Avanzar lleva signos **opuestos** y girar signos **iguales** — al revés que el
ejemplo de ellos, y consistente con el montaje espejado. David confirmó los
cuatro sentidos mirando las ruedas con el robot sobre un libro.

---

# 2. Resultados del simulador

Metodología: un cambio por vez, misma semilla en todas las comparaciones.
Los cambios 1 a 4 se midieron con 30 corridas; del 5 en adelante, con 100,
porque las diferencias eran de 2-3 puntos y 30 no alcanzaba para distinguirlas
del ruido.

```
py sim\pista.py --n 30 | Out-File -Encoding utf8 resultado.txt
```

## Primera tanda — 30 corridas, dificultad 0.2

| # | Cambio | 3 cubos | Prom. cubos | Mediana | Mejor | Peor |
|---|---|---|---|---|---|---|
| 1 | versión inicial | 22/30 (73%) | 2.67 | 89.5 s | 60.4 s | 154.1 s |
| 2 | TOLERANCIA_ENTREGA 2.0→1.0 + arreglo del bucle | 26/30 (87%) | 2.87 | 77.8 s | 51.2 s | 158.1 s |
| 3 | no contar intento si el cubo quedó casi dentro | 27/30 (90%) | 2.90 | 77.8 s | 51.2 s | 158.1 s |
| 4 | cuantizar el costo del reparto | 27/30 (90%) | 2.90 | 77.8 s | 51.2 s | 158.1 s |

## Segunda tanda — con la física real medida

El salto de 90% a 53% no es un retroceso: es que hasta el cambio 4 el
simulador usaba una velocidad inventada, mucho más rápida que la real. Al
poner los 2.7 celdas/s medidos, el problema se volvió el de verdad.

| # | Cambio | 3 cubos | Mediana | Peor |
|---|---|---|---|---|
| 5 | física real medida (2.7 celdas/s, 0.8 s por giro) | 16/30 (53%) | 216 s | 466 s |
| 6 | timeouts 25 s → 55 s | 23/30 (77%) | 260 s | 495 s |
| 7 | arreglo del arrastre al retroceder | 28/30 (93%) | 184.5 s | 263.7 s |
| 8 | `in_depot` del árbitro en vez de nuestra cuenta | 28/30 (93%) | 188.5 s | 257.1 s |

## Tercera tanda — 100 corridas

| # | Cambio | 3 cubos | Mediana | Peor |
|---|---|---|---|---|
| 8 | (línea base, la del cambio 8) | 96/100 (96%) | 186.9 s | 309.2 s |
| 9 | soltar el cubo que el reparto ya no me da | 96/100 (96%) | 183.4 s | 266.0 s |
| 10 | **arreglo del congelamiento por timeout** | **98/100 (98%)** | 184.9 s | 384.3 s |

El cambio 10 salió de un bug encontrado en el robot real, no en el simulador.
Sube el peor caso de 266 a 384 s, pero eso es una mejora: son dos corridas que
**antes fracasaban** y ahora terminan. Un éxito a los 384 s vale más que un
fracaso, porque el límite son 600.

---

# 3. Detalle de los cambios

## Cambio 2

**a) `TOLERANCIA_ENTREGA` de 2.0 a 1.0 celdas** (controller.py)

El robot frenaba demasiado pronto: el cubo quedaba corto y cada reintento lo
acercaba ~0.8 celdas. En el log se veía la escalera
`falta 5.34 → 3.59 → 2.85 → 2.25 → 1.19 → 0.73 → entra`: seis pasadas para una
entrega.

**b) Arreglo del bucle RETROCEDER ↔ VERIFICAR** (controller.py, estado VERIFICAR)

Al vencer el timeout de VERIFICAR se volvía a RETROCEDER, que reiniciaba su
punto de origen y retrocedía otras 5 celdas. Bucle infinito: las corridas 11,
13 y 23 se colgaron ahí y una terminó con 0 cubos.

```diff
- self._ir_a(RETROCEDER, ahora_ms)
+ self._abandonar(ahora_ms)
```

Ahora un intento fallido de verificación reinicia la maniobra completa.

## Cambio 3 — `FALTA_CASI_DENTRO` (controller.py)

En la corrida 28 aparecía `no entró: green falta 0.16` y a las dos líneas
`postergo green tras 3 intentos`: faltaban 3 mm y el cubo se mandaba al final
de la cola. El contador no distinguía entre "estuvo cerquísima" y "ni se
acercó".

Ahora `_abandonar` recibe cuánto faltó y no cuenta el intento si es menor a
1.0 celdas. Los timeouts siguen contando, porque ahí no hay dato de distancia.

Efecto: +1 corrida completa. El tiempo no cambia, y tiene sentido: el arreglo
no acelera las entregas, solo evita perder un cubo casi listo.

## Cambio 6 — timeouts de 25 a 55 segundos

Con la física real, 25 segundos no alcanzan para cruzar la cancha y alinearse.
El robot abandonaba maniobras que iban bien. 55 s cubre el peor caso medido.

## Cambio 7 — el arrastre al retroceder (sim/pista.py)

Era un bug **del simulador**, no del robot. `_empujar` movía el cubo con el
rover sin mirar la dirección, así que al retroceder lo arrastraba de vuelta y
lo sacaba de la zona. Era la causa dominante del patrón de reintentos en
escalera.

```python
# Producto punto: positivo = el rover se mueve HACIA el cubo.
if dcol * dx + drow * dy <= 0:
    continue
```

77% → 93%. Los brazos en V empujan, no agarran: un rover que retrocede suelta
el cubo.

## Cambio 8 — `in_depot`, el veredicto del árbitro

Desde el contrato v3 cada cubo trae `in_depot`: la decisión oficial. Nuestra
cuenta local era **más estricta** que la del juez — ventana conservadora de
115.2 × 65.1 mm contra 120.1 × 70.1 mm del árbitro, que agranda 2.5 mm por
lado.

Nos estábamos puntuando más duro que el torneo. El resultado no cambió (93%),
y eso es información: la ventana conservadora ya estaba bien calibrada. Pero
ahora el número está medido contra el mismo criterio del miércoles.

`world.cubo_en_zona` se conserva porque calcula **cuánto faltó**, que el
contrato no publica y es con lo que se ajusta `TOLERANCIA_ENTREGA`.

## Cambio 9 — soltar el cubo que el reparto ya no me da

Medido en el simulador con el reparto impreso en dos instantes:

```
t =    0 ms   reparto:  r10 → green        r11 → red, blue
t = 1500 ms   reparto:  r10 → red, blue    r11 → green   ← se dio vuelta
```

El rover 10 se acerca a green; como el makespan es `max(t_10, t_11)` y manda
el más lento, de pronto conviene que el 10 haga los dos lejanos. El 11 obedece
el reparto nuevo y toma green. Y el 10, pegado a green, veía la alternativa
más cara y **no lo soltaba**. Los dos sobre el mismo cubo y nadie sobre los
otros dos.

El error de diseño: **histéresis sobre una decisión local dentro de un reparto
que se recalcula global.** La histéresis rompía la garantía de partición.

```python
# El reparto ya no me da este cubo: lo suelto, sin histéresis.
if objetivo_actual not in mia:
    return nuevo
```

La histéresis queda solo para elegir entre cubos que **sí** son míos, que es
para lo que servía: evitar el thrashing entre dos objetivos empatados.

## Cambio 10 — el congelamiento por timeout

Ver la sección 4: salió del robot real.

---

# 4. Hipótesis descartadas

## Cuantizar el costo del reparto · SIN EFECTO

Hipótesis: los dos rovers eligen a veces el mismo cubo porque no reciben
exactamente el mismo mensaje, y un reparto casi empatado se resuelve distinto
en cada uno. Se agregó `CUANTO_COSTO_S = 1.0` para tratar como empate las
diferencias menores a un segundo.

Resultado: **idéntico, hasta el decimal.** La hipótesis era errada: los costos
entre repartos difieren en más de 1 segundo, así que redondear no cambia al
ganador.

## Los postergados privados · HIPÓTESIS FALSA

> Nota del 3-oct: esto estaba escrito como la causa probable de los repartos
> duplicados. **Es incorrecto y queda acá solo como registro.**

Se creía que cuando un rover posterga un cubo y el otro no se entera, los dos
corren el mismo algoritmo con entradas distintas y la coordinación implícita
se rompe. Se llegó a plantear agregar ESP-NOW para comunicar los rovers.

**Es falso.** Revisando `mi_objetivo`, los postergados solo **reordenan la
mitad propia** del reparto; nunca cambian el reparto en sí. Si el rover 10
posterga green, green sigue siendo del rover 10 — solo lo intenta de último. El
rover 11 nunca necesita saberlo porque nunca mira la mitad ajena.

La causa real era el cambio 9. **No hace falta ninguna comunicación entre
rovers.**

## Orden canónico por ID en `repartir` · SIN EFECTO, se deja

Hipótesis: ante un empate exacto de makespan, cada robot se ponía a sí mismo
como "A" y recorría las máscaras en orden espejado, así que el desempate
"gana el primero que apareció" los llevaba a mitades distintas.

Resultado: **salida byte por byte idéntica.** Se comprobó directamente que los
dos robots siempre llegaban al mismo reparto, con o sin el cambio. Los empates
exactos no ocurren con costos en punto flotante.

Se deja puesto igual: cuesta cero y cubre ese caso por construcción. El
cálculo ya no menciona "yo" en ningún lado.

## Congelar el reparto (recalcularlo solo al entrar un cubo) · PEOR

| | antes | congelado |
|---|---|---|
| 3 cubos | **96%** | 93% |
| mediana | **186.9 s** | 202.4 s |
| peor caso | 309.2 s | **239.1 s** |
| arranques duplicados | 28/100 | **0/100** |

Elimina por completo los arranques duplicados y recorta 70 s del peor caso,
pero cuesta 3 puntos de éxito y 15 s de mediana.

Resultó que los arranques duplicados eran **útiles**: cuando los dos rovers
caen sobre el mismo cubo, el segundo termina ayudando a meterlo, y recalcular
el reparto 20 veces por segundo funciona como una reasignación oportunista que
se adapta a cómo se van moviendo. **No se implementó.**

---

# 5. El robot real contra el mock_publisher

3-oct-2026. Primera vez que el sistema completo corrió en un ESP32.

| Medición | Valor |
|---|---|
| Hz del lazo, `code.py` normal | 22-24 Hz |
| Hz con el print del reparto (p6b) | 16-18 Hz |
| Hz sin objetivo (fase IDLE) | 39-41 Hz |
| Mensajes pisados por el publicador | 0 |
| Tiempo de reconexión tras caída | ~3 s |

La telemetría llega a 20 Hz, así que con 22-24 el robot nunca se atrasa. El
`pisados=0` lo confirma desde el otro lado: nunca hubo que descartarle un
mensaje por lento.

**El cálculo del reparto es la parte cara.** De 40 Hz sin objetivo a 24 con
objetivo: la fuerza bruta cuesta unos 16 Hz. Hay margen, pero no es gratis.

## Validación cruzada

La prueba de integración corrida en la laptop predijo, para el rover 10 desde
la posición de salida:

```
objetivo = green | v = +0.450 | w = +0.044
```

El robot real, contra el mock, dio:

```
28.5Hz | READY | yo=(4.0,17.4) | estado=IR_APROX | obj=green | v=+0.45 w=+0.05
```

Mismo cubo, misma velocidad, mismo giro. **El código se comporta igual en el
ESP32 que en la laptop**, que es lo que permite confiar en las 100 corridas del
simulador.

## Coordinación verificada en hardware

Los dos robots corriendo a la vez, cada uno imprimiendo su versión del reparto:

```
ROBOT 10:  reparto: 10 -> ['green'] | 11 -> ['blue', 'red'] | companero visible: si
ROBOT 11:  reparto: 11 -> ['blue', 'red'] | 10 -> ['green'] | companero visible: si
```

Mismo contenido, distinto orden porque cada uno se nombra primero a sí mismo.
Estable en todos los cuadros, sin oscilar. **Dos máquinas separadas, sin un
solo mensaje entre ellas, llegando al mismo reparto.**

También quedó verificado el retardo de salida: el rover 11 pasa ~3 segundos en
`ESPERANDO` después de que la fase ya es `READY`, mientras el 10 arranca al
instante.

## Bugs encontrados al correr en hardware

Ninguno de los cuatro era detectable desde la laptop.

### 1. `math.hypot` no existe en CircuitPython

`world.distancia()` la usaba en cada ciclo. En la laptop corre Python normal,
donde sí existe, así que el simulador nunca lo vio. En el robot habría muerto
en el primer ciclo con `AttributeError`.

El módulo `math` de CircuitPython trae `sqrt`, `atan2`, `degrees`, `radians`,
`cos`, `sin` — pero no `hypot`. Documentado en docs.circuitpython.org.
Reemplazada por `sqrt(dx² + dy²)` escrita a mano.

### 2. Congelamiento al vencerse un timeout

`_abandonar()` terminaba llamando a `_ir_a(IR_APROX)`, que **no hace nada** si
ya estábamos en IR_APROX. El cronómetro del estado nunca se reiniciaba, así que
el timeout seguía vencido: al ciclo siguiente volvía a vencer, volvía a
abandonar, y a los tres ciclos postergaba otro cubo. Para siempre, 25 veces por
segundo, devolviendo `(0, 0)`.

El robot quedaba **congelado el resto de la ronda**. En el log:

```
postergo blue tras 3 intentos
postergo blue tras 3 intentos
postergo blue tras 3 intentos
24.9Hz | RUNNING | estado=IR_APROX | obj=blue | v=+0.00 w=+0.00
```

Se vio porque el mock mueve sus rovers por su cuenta y el timeout salta de
verdad; en el simulador el rover llega al punto antes de que venza. **También
subió el simulador de 96% a 98%.**

### 3. No detectaba la caída de la conexión

Con el timeout de 10 ms, `recv_into` levanta `OSError` tanto cuando el servidor
cerró como cuando todavía no hay datos. Indistinguibles desde el código.

El robot estuvo **106 segundos** repitiendo `sin telemetria | edad=...` sin
intentar reconectar ni una vez.

Arreglado reconectando por silencio: la visión publica a 20 Hz, así que 3 s sin
datos son 60 mensajes perdidos. Eso no es una pausa, es una conexión muerta.

### 4. Fuga de sockets ← el más grave

Cada intento de conexión fallido creaba un socket y **no lo cerraba**. El ESP32
tiene 4-8 en total. A los pocos reintentos: `Out of sockets`, y la placa queda
incapaz de abrir uno más aunque el servidor esté perfecto.

Solo se recupera **reiniciando la placa** — y el reglamento 4.5 prohíbe tocar
los robots después del `READY`.

El escenario del torneo: la organización levanta su sistema de visión con los
robots ya encendidos. Si tarda unos segundos de más, el robot gasta sus sockets
reintentando y queda muerto permanentemente, sin derecho a repetición
(reglamento 10.7: una falla de comunicación del equipo no es falla de
infraestructura).

Arreglado con una función `_cerrar()` llamada en **todos** los caminos de error.

### Bonus: el orden de los timeouts del socket

`settimeout(0.01)` antes del `connect()` le da 10 ms al handshake TCP, que por
wifi tarda entre 5 y 50 ms. Hay que conectar con 5 s y recién después bajar a
10 ms para recibir. No fue la causa del fallo de ese día (era el firewall), pero
habría fallado solo con la red cargada.

## Lo que quedó verificado del reglamento

| Regla | Qué dice | Estado |
|---|---|---|
| 4.3 | READY debe disparar la estrategia automáticamente | ✅ dos veces |
| 4.6 | Prohibido usar botones para arrancar | ✅ no se usa ninguno |
| 4.8 | IDLE → READY → RUNNING | ✅ ciclo completo |
| — | El robot frena al perder telemetría | ✅ |

---

# 6. Red

| Dato | Valor |
|---|---|
| Wifi | 2.4 GHz obligatorio — el ESP32 no ve las de 5 GHz |
| Puerto | 2026 TCP, NDJSON, 20 Hz |
| Robot 10 | 192.168.100.216 |
| Robot 11 | 192.168.100.217 |
| Laptop de visión (3-oct) | 192.168.100.3 |

**El firewall de la laptop bloquea el 2026 por defecto.** `ufw` estaba activo
con solo el 1433 abierto, y los paquetes del robot se descartaban en silencio.

El error lo delata: `ETIMEDOUT` significa que **nadie contesta** (paquete
descartado). `ECONNREFUSED` sería "llegué pero el puerto está cerrado". Un
firewall que DROPea da timeout; uno que rechaza, refused.

```
sudo ufw allow 2026/tcp
```

Diagnóstico de cuatro comandos, en orden:

```
iwgetid -r                      # ¿misma red que el robot?
ping -c 3 192.168.100.216       # ¿se ven? decisivo
sudo ufw status                 # ¿firewall?
ss -tlnp | grep 2026            # ¿escucha en 0.0.0.0?
```

Si el ping responde, el problema es el puerto. Si no responde, es la red y
ningún cambio de código lo arregla.

---

# 7. Pendiente de medir en la cancha

| Qué | Por qué importa |
|---|---|
| **El sentido de `theta`** | Si crece horario en vez de antihorario, cada giro sale invertido y el lazo **agranda** el error en vez de corregirlo: el robot entra en un giro cada vez más rápido y nunca se alinea |
| `DISTANCIA_APROXIMACION` (6.5 celdas) | Salió del simulador, nunca se probó con un cubo real |
| `TOLERANCIA_ENTREGA` (1.0 celdas) | Ídem |
| Fricción real del cubo al empujarlo | El simulador la supone |
| Duración de las pilas | Cuántas rondas completas aguantan |

## Sin implementar

**Protección de zonas ya entregadas.** Un rover que va de paso hacia otro cubo
y atraviesa una zona con un cubo adentro lo saca, y eso borra el cubo **y su
tiempo** (reglamento 7.4 y 8.8). El código ya evita empujar de más —corta el
empuje apenas `in_depot` da true— pero no evita pasar por encima.

Es lo único de la lista de arreglables que quedó pendiente. Se puede atacar con
datos reales de cuánto ocurre.