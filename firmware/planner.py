"""planner.py — decide a dónde ir.

Dos responsabilidades:
  1. Repartir los cubos entre los dos rovers.
  2. Calcular a qué punto físico ir para empujar un cubo.

Sin hardware: se prueba en la laptop con py, igual que world.py.

COORDINACIÓN IMPLÍCITA
----------------------
Los dos rovers reciben la MISMA telemetría y corren ESTE MISMO código, así
que llegan al mismo reparto sin hablarse. Cada uno mira qué le tocó a sí
mismo y arranca.

Para que eso funcione, el algoritmo tiene que ser determinista: nada de azar,
y los empates se rompen siempre igual. De ahí que COLORES tenga un orden fijo
y que las combinaciones se recorran siempre en el mismo orden. Si cada rover
desempatara distinto, los dos irían por el mismo cubo.
"""

import math

import world

# Orden fijo. No es estético: es lo que hace que los empates se rompan igual
# en los dos robots.
COLORES = ("blue", "green", "red")


# --------------------------------------------------------------------------
# Constantes de maniobra  [AJUSTAR con el robot en la cancha]
# --------------------------------------------------------------------------

# A qué distancia detrás del cubo se ubica el punto de aproximación.
# Tiene que dejar espacio para girar sin golpear el cubo. Este valor sale de
# probarlo en el simulador; en la cancha real hay que reajustarlo.
DISTANCIA_APROXIMACION = 6.5    # celdas = 130 mm

# Velocidad estimada, solo para COMPARAR repartos. No se usa para navegar.
VELOCIDAD_CELDAS_S = 2.7        # medido: 27 cm en 5 s a throttle 0.45
SEGUNDOS_POR_GIRO_90 = 0.8      # medido a throttle 0.30


# --------------------------------------------------------------------------
# Punto de aproximación
# --------------------------------------------------------------------------

def punto_aproximacion(cubo, depot, grid=None):
    """Dónde pararse para empujar el cubo hacia su zona.

    NUNCA se va directo al cubo. Se va al punto que está sobre la recta
    zona→cubo, unos centímetros detrás del cubo. Desde ahí, empujar en línea
    recta lo lleva al destino.

        [zona] ← [cubo] ← [punto de aproximación] ← [rover]

    Si uno va directo al cubo, lo empuja en la dirección en que venía, que
    casi nunca es la correcta. Es el mismo truco que usan en RoboCup para
    patear al arco: primero te ubicás detrás de la pelota, alineado con el
    arco, y recién ahí avanzás.

    Devuelve (col, row, rumbo), donde rumbo es hacia dónde hay que mirar
    parado en ese punto.
    """
    # Vector del cubo hacia su zona.
    dcol = depot["col"] - cubo["col"]
    drow = depot["row"] - cubo["row"]
    largo = math.sqrt(dcol * dcol + drow * drow)

    if largo < 1e-6:
        # El cubo ya está en el centro de la zona: no hay dirección que seguir.
        return cubo["col"], cubo["row"], 0.0

    # Unitario: el mismo vector pero de largo 1. Multiplicándolo por una
    # distancia se obtiene un punto a esa distancia en esa dirección.
    ucol, urow = dcol / largo, drow / largo

    # El punto va en sentido CONTRARIO (restando), o sea del otro lado del
    # cubo respecto a la zona.
    distancia_util = DISTANCIA_APROXIMACION
    col = cubo["col"] - ucol * distancia_util
    row = cubo["row"] - urow * distancia_util

    # Si el cubo quedó pegado al borde opuesto a su zona, el punto cae FUERA
    # de la cancha. Un objetivo inalcanzable deja al rover empujando contra la
    # pared hasta que salta el timeout, así que se acorta hasta que entre.
    if grid is not None:
        margen = 2.0
        while distancia_util > 1.0 and not (
                margen <= col <= grid["cols"] - margen and
                margen <= row <= grid["rows"] - margen):
            distancia_util -= 0.5
            col = cubo["col"] - ucol * distancia_util
            row = cubo["row"] - urow * distancia_util

    rumbo = world.rumbo_hacia(col, row, depot["col"], depot["row"])
    return col, row, rumbo


# --------------------------------------------------------------------------
# Costo de una tarea
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# Dónde estacionar cuando ya no queda nada que hacer
# --------------------------------------------------------------------------
# [MEDIDO 5-oct-2026, simulador a dificultad 0.8] El robot que terminaba su
# parte se quedaba parado donde había entregado el último cubo: al lado de
# una zona, que es justo por donde pasan los empujes. En las rondas que se
# perdían, estaba a menos de 2 celdas de la línea de empuje del cubo que le
# faltaba al compañero, y el compañero le cedía el paso, volvía, le cedía
# otra vez... para siempre.
#
# Se estaciona en una de las cuatro esquinas, metida hacia adentro: la que
# queda más lejos de todo el trabajo pendiente. Las zonas están en el medio
# de los lados, así que las esquinas casi nunca quedan en el camino.
#
# 7.5 celdas desde el borde: el robot mide ~2.9 de radio y el marcador de la
# esquina ~2.5. Más cerca, el robot podría tapar el marcador, y sin cuatro
# esquinas la visión pierde las coordenadas de TODA la cancha.
RETIRO_ESQUINA = 7.5   # celdas


def _distancia_a_tramo(px, py, ax, ay, bx, by):
    """Distancia del punto P al segmento AB."""
    dx, dy = bx - ax, by - ay
    largo2 = dx * dx + dy * dy
    if largo2 < 1e-9:
        return world.distancia(px, py, ax, ay)
    t = ((px - ax) * dx + (py - ay) * dy) / largo2
    t = max(0.0, min(1.0, t))
    return world.distancia(px, py, ax + t * dx, ay + t * dy)


def lugar_para_estacionar(mundo):
    """La esquina más lejana de los empujes pendientes, o None si no hay.

    Un empuje pendiente ocupa el tramo que va de su punto de aproximación a
    su zona: por ahí pasa el robot que lo hace y por ahí pasa el cubo.
    """
    grid = mundo.msg["grid"]
    tramos = []
    for color in COLORES:
        if color not in mundo.pendientes():
            continue
        cubo, depot = mundo.cubo(color), mundo.depot(color)
        if cubo is None or depot is None:
            continue
        ac, ar, _ = punto_aproximacion(cubo, depot, grid)
        tramos.append((ac, ar, depot["col"], depot["row"]))
    if not tramos:
        return None

    k = RETIRO_ESQUINA
    cols, rows = grid["cols"], grid["rows"]
    esquinas = ((k, k), (cols - k, k), (k, rows - k), (cols - k, rows - k))
    mejor, mejor_margen = None, -1.0
    for (ec, er) in esquinas:          # orden fijo: desempata igual siempre
        margen = min(_distancia_a_tramo(ec, er, *t) for t in tramos)
        if margen > mejor_margen + 1e-9:
            mejor, mejor_margen = (ec, er), margen
    return mejor


def _costo_cubo(col, row, cubo, depot):
    """Segundos estimados para entregar un cubo desde una posición dada.

    Es una ESTIMACIÓN para comparar repartos, no una predicción. Alcanza con
    que ordene bien las opciones: si el reparto A es mejor que el B, la cuenta
    tiene que decir eso, aunque los segundos exactos estén errados.

    Devuelve (segundos, col_final, row_final), porque si el rover encadena
    dos cubos, el segundo arranca donde terminó el primero.
    """
    aprox_col, aprox_row, _ = punto_aproximacion(cubo, depot)

    d_ir = world.distancia(col, row, aprox_col, aprox_row)
    d_empujar = world.distancia(cubo["col"], cubo["row"], depot["col"], depot["row"])

    t = (d_ir + d_empujar) / VELOCIDAD_CELDAS_S

    # Girar cuesta tiempo y la distancia no lo refleja. Se penaliza el giro de
    # alineación, que siempre ocurre.
    t += SEGUNDOS_POR_GIRO_90 * 1.5

    return t, depot["col"], depot["row"]


# No pisar lo que uno mismo ya entregó  (reglamento 7.4 y 8.8)
#
# [MEDIDO 5-oct-2026, simulador a dificultad 0.8] En las rondas que
# terminaban con CERO cubos después de haber entregado alguno, siempre era lo
# mismo: un robot entregaba su primer cubo y, empujando el segundo, pasaba a
# 2 celdas del que acababa de dejar y lo sacaba de la zona. Un cubo que sale
# de su zona se pierde con su tiempo. El orden de los dos cubos se elegía
# solo por distancia, sin mirar si el segundo empuje cruzaba la primera zona.
#
# Ahora un orden así se cobra caro. 6 celdas = radio del robot (2.9) + medio
# cubo (1.5) + margen: más cerca que eso, el cuerpo del robot lo toca.
DISTANCIA_PISAR = 6.0         # celdas
CASTIGO_PISAR_S = 60.0        # lo que cuesta, a ojo, perder y rehacer una entrega


def costo_lista(col, row, colores, mundo):
    """Segundos que tarda un rover en hacer una lista de cubos, en orden.

    Incluye el castigo por los órdenes en que un empuje posterior pasaría por
    encima de una zona que la misma lista ya llenó.
    """
    total = 0.0
    grid = mundo.msg["grid"] if mundo.msg else None
    llenas = []                    # zonas que esta lista ya dejó con su cubo
    for color in colores:
        cubo = mundo.cubo(color)
        depot = mundo.depot(color)
        if cubo is None or depot is None:
            continue
        t, col, row = _costo_cubo(col, row, cubo, depot)
        total += t
        if llenas:
            ac, ar, _ = punto_aproximacion(cubo, depot, grid)
            for (zc, zr) in llenas:
                if _distancia_a_tramo(zc, zr, ac, ar,
                                      depot["col"], depot["row"]) < DISTANCIA_PISAR:
                    total += CASTIGO_PISAR_S
        llenas.append((depot["col"], depot["row"]))
    return total


# --------------------------------------------------------------------------
# Reparto por fuerza bruta
# --------------------------------------------------------------------------

_CACHE_PERM = {}


def _permutaciones(lista):
    """Todas las ordenaciones posibles de una lista.

    Se escribe a mano porque CircuitPython no trae itertools completo.

    MEMORIZADA a propósito. repartir() la llama 24 veces por ciclo y el lazo
    corre a decenas de hertz, así que la versión recursiva estaba creando y
    tirando miles de listas por segundo en un ESP32 con muy poca RAM. Los
    colores son tres y fijos: hay a lo sumo 8 listas distintas que pedir, así
    que se calculan una vez y se devuelven siempre las mismas.

    El que llama NO debe modificar lo que recibe. Hoy nadie lo hace: repartir()
    solo las recorre y copia con list() lo que se guarda como mejor.
    """
    clave = tuple(lista)
    guardado = _CACHE_PERM.get(clave)
    if guardado is not None:
        return guardado

    if len(lista) <= 1:
        salida = [list(lista)]
    else:
        salida = []
        for i in range(len(lista)):
            resto = list(lista[:i]) + list(lista[i + 1:])
            for p in _permutaciones(resto):
                salida.append([lista[i]] + p)

    _CACHE_PERM[clave] = salida
    return salida


# Cuánto tiempo sin ver al compañero antes de darlo por AUSENTE y hacer su
# parte. Más corto, un parpadeo de la cámara haría que los dos fueran por el
# mismo cubo; más largo, un robot muerto en el torneo se lleva minutos de
# trabajo del otro. El domingo hubo huecos de detección de hasta 19 s con el
# robot VIVO, así que 5 s no lo cubre todo — pero tapado más de eso ya no se
# sabe dónde está, y esperarlo cuesta más que arriesgar un cubo compartido.
EDAD_COMPANERO_AUSENTE_MS = 5000


def _mejor_orden(pos, colores, mundo):
    """El orden más rápido para hacer todos esos cubos uno solo."""
    mejor, mejor_t = [], None
    for orden in _permutaciones(colores):
        t = costo_lista(pos["col"], pos["row"], orden, mundo)
        if mejor_t is None or t < mejor_t - 1e-9:
            mejor, mejor_t = list(orden), t
    return mejor


def repartir(mundo):
    """Devuelve {id_rover: [colores en orden]} minimizando el MAKESPAN.

    Makespan = cuándo termina el que termina ÚLTIMO. No la suma.

    La ronda se acaba cuando entra el último cubo, así que un reparto de
    100 s y 40 s pierde contra uno de 75 s y 70 s, aunque este haga más
    trabajo total: en el primero hay un rover parado 60 segundos sin sumar
    nada.

    Con 3 cubos y 2 rovers hay 2³ = 8 repartos, por sus ordenaciones: una
    docena de casos. La fuerza bruta da el óptimo y cabe en un for. Un método
    húngaro daría la misma respuesta con diez veces más código.
    """
    pendientes = [c for c in COLORES if c in mundo.pendientes()]

    yo = mundo.yo()
    if yo is None:
        return {mundo.mi_id: [], mundo.id_companero: []}

    # ------------------------------------------------------------------
    # ¿HAY COMPAÑERO?
    # ------------------------------------------------------------------
    # Antes, si la cámara no veía al compañero, se lo suponía parado en la
    # salida y se le seguían asignando cubos. Con un robot solo en la cancha
    # —o con el otro sin batería en pleno torneo— eso dejaba cubos asignados a
    # un robot que no existe: el que quedaba entregaba su parte, se ponía en
    # LISTO y los demás cubos se quedaban en la cancha. Medido el 5-oct:
    #     solo el 10 en la cancha -> {10: ['blue'], 11: ['green', 'red']}
    #
    # Ahora hay dos casos distintos:
    #   - tapado un rato (menos de EDAD_COMPANERO_AUSENTE_MS): sigue en el
    #     reparto, en su ÚLTIMA posición conocida. Sacarlo por un parpadeo de
    #     la cámara haría que los dos fueran por el mismo cubo.
    #   - ausente (no aparece, o lleva más que eso sin verse): fuera del
    #     reparto. Hago yo todo lo que queda.
    otro = mundo.rover(mundo.id_companero)
    hay_companero = (otro is not None and
                     otro.get("age_ms", 0) <= EDAD_COMPANERO_AUSENTE_MS)
    if not hay_companero:
        return {mundo.mi_id: _mejor_orden(yo, pendientes, mundo),
                mundo.id_companero: []}

    # ------------------------------------------------------------------
    # ORDEN CANÓNICO POR ID
    # ------------------------------------------------------------------
    # El cálculo no menciona "yo" en ningún lado: el de id menor es SIEMPRE A.
    # Así los dos robots recorren el mismo espacio en el mismo orden y llegan
    # al mismo resultado incluso ante un empate exacto de punto flotante, que
    # es el único caso que el orden anterior no cubría.
    if mundo.mi_id < mundo.id_companero:
        id_a, pos_a = mundo.mi_id, yo
        id_b, pos_b = mundo.id_companero, otro
    else:
        id_a, pos_a = mundo.id_companero, otro
        id_b, pos_b = mundo.mi_id, yo

    # ------------------------------------------------------------------
    # LOS DOS ROVERS TIENEN QUE TRABAJAR  (reglamento 12.2.13, 4-oct-2026)
    # ------------------------------------------------------------------
    # "Para completar válidamente los tres cubos, CADA rover deberá haber
    # transportado o empujado al menos un cubo hacia su zona de acopio."
    #
    # O sea que un reparto 3-0 no es una optimización agresiva: es un intento
    # INVÁLIDO. Aunque entren los tres cubos, no cuenta.
    #
    # Por eso, mientras queden dos o más cubos, se descartan los repartos que
    # dejen a alguien sin nada. Si el compañero está AUSENTE ni se llega acá
    # (ver arriba): tres cubos entregados por uno solo valen más que cero, y a
    # esa altura el intento ya estaba perdido de todas formas.
    exigir_ambos = len(pendientes) >= 2

    mejor = None

    # Cada bit de la máscara dice a qué rover va ese cubo.
    #   mascara = 0b000 -> los tres al rover A (el de id menor)
    #   mascara = 0b101 -> el primero y el tercero al B, el segundo al A
    for mascara in range(1 << len(pendientes)):
        lista_a, lista_b = [], []
        for i, color in enumerate(pendientes):
            if mascara & (1 << i):
                lista_b.append(color)
            else:
                lista_a.append(color)

        if exigir_ambos and (not lista_a or not lista_b):
            continue

        # Y para cada reparto, en qué orden los hace cada uno.
        for orden_a in _permutaciones(lista_a):
            for orden_b in _permutaciones(lista_b):
                t_a = costo_lista(pos_a["col"], pos_a["row"], orden_a, mundo)
                t_b = costo_lista(pos_b["col"], pos_b["row"], orden_b, mundo)

                makespan = max(t_a, t_b)   # el que termina último manda

                # El "menor estricto" es lo que rompe los empates de forma
                # estable: ante dos repartos iguales, gana el primero que
                # apareció, y como el orden de recorrido es fijo, los dos
                # robots eligen el mismo.
                if mejor is None or makespan < mejor[0] - 1e-9:
                    mejor = (makespan, list(orden_a), list(orden_b))

    if mejor is None:
        return {mundo.mi_id: [], mundo.id_companero: []}

    return {id_a: mejor[1], id_b: mejor[2]}


def mi_objetivo(mundo, objetivo_actual=None, postergados=None):
    """Qué cubo me toca ahora.

    Combina dos cosas que se pelean entre sí: el reparto global (que cambia a
    medida que los rovers se mueven) y el compromiso local (no cambiar de
    objetivo a mitad de camino).

    HISTÉRESIS: una vez comprometido con un cubo no se cambia, salvo que la
    alternativa sea claramente mejor. Sin esto el rover oscila entre dos
    objetivos casi empatados y no llega a ninguno: arranca hacia el verde, en
    el mensaje siguiente el rojo parece mejor por un pelo, gira, y así para
    siempre. En control eso se llama thrashing.

    PERO la histéresis NO puede aplicarse a un cubo que el reparto ya le dio al
    compañero. Medido en el simulador:

        t =    0 ms   reparto:  r10 → green        r11 → red, blue
        t = 1500 ms   reparto:  r10 → red, blue    r11 → green   ← se dio vuelta

    El rover 10 se acercó a green, y como el makespan es max(t_10, t_11) lo que
    manda es el más lento: de pronto conviene que el 10 haga los dos lejanos. El
    11 obedece el reparto nuevo y toma green. Y el 10, pegado a green, veía la
    alternativa más cara y NO lo soltaba. Los dos sobre el mismo cubo y nadie
    sobre los otros dos.

    De ahí la regla de abajo: si el reparto ya no me da este cubo, lo suelto sin
    discutir. La histéresis queda solo para lo que de verdad es mío.

    `postergados` son cubos que ya fallaron varias veces: se dejan para el
    final, pero no se descartan, porque si al final es lo único que queda hay
    que volver a intentarlo. Solo reordenan MI mitad, nunca cambian el reparto,
    así que no hace falta que el compañero los conozca.
    """
    reparto = repartir(mundo)
    mia = reparto.get(mundo.mi_id, [])

    if not mia:
        return None

    if postergados:
        preferidos = [c for c in mia if c not in postergados]
        mia = preferidos + [c for c in mia if c in postergados]

    # Un cubo que la cámara no ve hace rato va al final de la fila. No se
    # descarta —sigue pendiente y puede ser lo único que quede— pero no se
    # elige mientras haya otro que SÍ se esté viendo: su posición publicada
    # es la de hace cinco segundos o más, y el punto de aproximación que sale
    # de ahí apunta a donde el cubo ya no está.
    viejos = [c for c in mia
              if (mundo.edad_cubo(c) or 0) > world.EDAD_MAXIMA_CUBO_MS]
    if viejos and len(viejos) < len(mia):
        mia = [c for c in mia if c not in viejos] + viejos

    nuevo = mia[0]

    # Sin objetivo previo, o el previo ya se entregó: tomamos el nuevo.
    if objetivo_actual is None or objetivo_actual not in mundo.pendientes():
        return nuevo
    if nuevo == objetivo_actual:
        return objetivo_actual

    # El reparto ya no me da este cubo: lo suelto, sin histéresis. Es el
    # compañero el que lo tiene asignado, y si los dos insistimos en el mismo
    # queda otro sin atender.
    if objetivo_actual not in mia:
        return nuevo

    # Sigue siendo mío, solo cambió el orden: cambiar únicamente si la
    # alternativa es 15% mejor. Ese margen es la histéresis.
    yo = mundo.yo()
    costo_actual = costo_lista(yo["col"], yo["row"], [objetivo_actual], mundo)
    costo_nuevo = costo_lista(yo["col"], yo["row"], [nuevo], mundo)
    if costo_nuevo < costo_actual * 0.85:
        return nuevo
    return objetivo_actual