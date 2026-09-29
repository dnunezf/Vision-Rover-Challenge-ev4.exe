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
VELOCIDAD_CELDAS_S = 6.0        # [MEDIR con el robot]
SEGUNDOS_POR_GIRO_90 = 1.2      # [MEDIR con el robot]


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


def costo_lista(col, row, colores, mundo):
    """Segundos que tarda un rover en hacer una lista de cubos, en orden."""
    total = 0.0
    for color in colores:
        cubo = mundo.cubo(color)
        depot = mundo.depot(color)
        if cubo is None or depot is None:
            continue
        t, col, row = _costo_cubo(col, row, cubo, depot)
        total += t
    return total


# --------------------------------------------------------------------------
# Reparto por fuerza bruta
# --------------------------------------------------------------------------

def _permutaciones(lista):
    """Todas las ordenaciones posibles de una lista.

    Se escribe a mano porque CircuitPython no trae itertools completo.
    """
    if len(lista) <= 1:
        return [list(lista)]
    salida = []
    for i in range(len(lista)):
        resto = list(lista[:i]) + list(lista[i + 1:])
        for p in _permutaciones(resto):
            salida.append([lista[i]] + p)
    return salida


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

    otro = mundo.companero()
    if otro is None:
        # Si la cámara no ve al compañero en este cuadro, asumimos que está en
        # la salida. Es mejor que dejarlo fuera del reparto y cargarnos todo.
        otro = {"col": mundo.msg["start"]["col"],
                "row": mundo.msg["start"]["row"]}

    mejor = None

    # Cada bit de la máscara dice a qué rover va ese cubo.
    #   mascara = 0b000 -> los tres al rover A
    #   mascara = 0b101 -> el primero y el tercero al B, el segundo al A
    for mascara in range(1 << len(pendientes)):
        lista_a, lista_b = [], []
        for i, color in enumerate(pendientes):
            if mascara & (1 << i):
                lista_b.append(color)
            else:
                lista_a.append(color)

        # Y para cada reparto, en qué orden los hace cada uno.
        for orden_a in _permutaciones(lista_a):
            for orden_b in _permutaciones(lista_b):
                t_a = costo_lista(yo["col"], yo["row"], orden_a, mundo)
                t_b = costo_lista(otro["col"], otro["row"], orden_b, mundo)

                makespan = max(t_a, t_b)   # el que termina último manda

                # El "menor estricto" es lo que rompe los empates de forma
                # estable: ante dos repartos iguales, gana el primero que
                # apareció, y como el orden de recorrido es fijo, los dos
                # robots eligen el mismo.
                if mejor is None or makespan < mejor[0] - 1e-9:
                    mejor = (makespan, list(orden_a), list(orden_b))

    if mejor is None:
        return {mundo.mi_id: [], mundo.id_companero: []}

    return {mundo.mi_id: mejor[1], mundo.id_companero: mejor[2]}


def mi_objetivo(mundo, objetivo_actual=None, postergados=None):
    """Qué cubo me toca ahora, con histéresis.

    Una vez comprometido con un cubo no se cambia, salvo que la alternativa
    sea claramente mejor. Sin esto el rover oscila entre dos objetivos casi
    empatados y no llega a ninguno: arranca hacia el verde, en el mensaje
    siguiente el rojo parece mejor por un pelo, gira, y así para siempre. En
    control eso se llama thrashing.

    `postergados` son cubos que ya fallaron varias veces: se dejan para el
    final, pero no se descartan, porque si al final es lo único que queda hay
    que volver a intentarlo.
    """
    reparto = repartir(mundo)
    mia = reparto.get(mundo.mi_id, [])

    if not mia:
        return None

    if postergados:
        preferidos = [c for c in mia if c not in postergados]
        mia = preferidos + [c for c in mia if c in postergados]

    nuevo = mia[0]

    # Sin objetivo previo, o el previo ya se entregó: tomamos el nuevo.
    if objetivo_actual is None or objetivo_actual not in mundo.pendientes():
        return nuevo
    if nuevo == objetivo_actual:
        return objetivo_actual

    # Cambiar solo si la alternativa es 15% mejor. Ese margen es la histéresis.
    yo = mundo.yo()
    costo_actual = costo_lista(yo["col"], yo["row"], [objetivo_actual], mundo)
    costo_nuevo = costo_lista(yo["col"], yo["row"], [nuevo], mundo)
    if costo_nuevo < costo_actual * 0.85:
        return nuevo
    return objetivo_actual