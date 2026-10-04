"""world.py — geometría y modelo del mundo.

Traduce el JSON crudo de la visión en respuestas útiles.

No importa nada de hardware a propósito. Así este mismo archivo corre en el
ESP32 y en la laptop, y se puede probar con py sin tener el robot enfrente.

Regla: el resto del código NUNCA toca el diccionario del mensaje. Todo pasa
por acá. Si mañana el contrato cambia un nombre de campo, se arregla en un
solo lugar.

CONTRATO v3 (octubre 2026)
-------------------------
Cada cubo trae `in_depot`: el veredicto del árbitro. Es lo que cuenta los
cubos y cierra la ronda, así que manda sobre cualquier cuenta nuestra.
"""

import math


def _hipotenusa(dx, dy):
    """sqrt(dx² + dy²), escrito a mano.

    CircuitPython NO trae math.hypot. Su módulo math tiene sqrt, atan2,
    degrees, radians, cos y sin, pero no hypot — está documentado en
    docs.circuitpython.org. En la laptop funciona porque ahí corre Python
    normal, así que el simulador nunca lo detecta: el error aparece recién en
    el robot, en el primer ciclo, y tumba todo.
    """
    return math.sqrt(dx * dx + dy * dy)


# --------------------------------------------------------------------------
# Geometría
# --------------------------------------------------------------------------

def normalizar(angulo):
    """Lleva un ángulo al rango (-180, 180].

    Hace falta porque los errores angulares se calculan restando, y una resta
    puede dar 350 grados cuando en realidad el giro corto es de -10. Sin esto,
    el robot daría la vuelta larga.
    """
    while angulo > 180.0:
        angulo -= 360.0
    while angulo <= -180.0:
        angulo += 360.0
    return angulo


def rumbo_hacia(col_a, row_a, col_b, row_b):
    """Ángulo en grados desde A hacia B, en la convención del contrato.

    theta = 0 apunta hacia donde crece col, y crece en sentido ANTIHORARIO.
    Pero row crece hacia ABAJO, como en cualquier imagen.

    De ahí el signo negativo en la resta de filas: sin él, el robot gira para
    el lado contrario y uno pasa horas culpando a la cámara.
    """
    return math.degrees(math.atan2(-(row_b - row_a), col_b - col_a)) % 360.0


def distancia(col_a, row_a, col_b, row_b):
    """Distancia en celdas entre dos puntos. Para pasar a mm: × cell_mm."""
    return _hipotenusa(col_b - col_a, row_b - row_a)


def adelante(col, row, theta, celdas):
    """El punto que está `celdas` al frente de una pose.

    Se usa para calcular hacia dónde vamos a terminar si seguimos derecho, y
    para ubicar el punto donde debería estar el cubo que llevamos entre los
    brazos cuando la cámara no lo reporta fresco.
    """
    rad = math.radians(theta)
    return col + math.cos(rad) * celdas, row - math.sin(rad) * celdas


def lado_mas_cercano(col, row, cols, rows):
    """Sobre qué borde de la cancha apoya una zona de acopio.

    El mensaje no lo dice: hay que deducirlo de la posición del centro. Se
    necesita para saber si la zona es ancha de lado a lado (arriba/abajo) o
    alta (izquierda/derecha).
    """
    opciones = (
        ("arriba", row),
        ("abajo", rows - row),
        ("izquierda", col),
        ("derecha", cols - col),
    )
    return min(opciones, key=lambda par: par[1])[0]


def cubo_en_zona(cubo, depot, depot_size, grid, cube_side):
    """Devuelve (está_adentro, cuánto_falta_en_celdas).

    Desde la v3 esta NO es la cuenta oficial: el veredicto viaja en
    `in_depot`. Queda por dos motivos.

    Primero, `falta` sigue siendo útil: dice CUÁNTO le faltó al cubo cuando
    no entró, y eso el contrato no lo publica. Es el número con el que se
    ajusta TOLERANCIA_ENTREGA.

    Segundo, es el respaldo para grabaciones viejas en v2, que no traen
    `in_depot`.

    Ojo: esta cuenta es MÁS ESTRICTA que la del árbitro. Usa media diagonal
    del cubo como margen (el peor giro posible), mientras que el árbitro
    agranda su ventana 2.5 mm por lado. Ventana conservadora: 115.2 × 65.1 mm.
    Ventana del árbitro: 120.1 × 70.1 mm. Por eso conviene creerle a él.
    """
    lado = lado_mas_cercano(depot["col"], depot["row"], grid["cols"], grid["rows"])

    # La zona es un rectángulo de 10 × 7.5 celdas. Cuál medida va en cada eje
    # depende de contra qué borde está apoyada.
    if lado in ("arriba", "abajo"):
        semi_col = depot_size["length"] / 2.0
        semi_row = depot_size["depth"] / 2.0
    else:
        semi_col = depot_size["depth"] / 2.0
        semi_row = depot_size["length"] / 2.0

    margen = cube_side * math.sqrt(2.0) / 2.0

    # Cuánto se pasa el cubo de la zona útil en cada eje. Si no se pasa, 0.
    exceso_col = max(0.0, abs(cubo["col"] - depot["col"]) - (semi_col - margen))
    exceso_row = max(0.0, abs(cubo["row"] - depot["row"]) - (semi_row - margen))

    falta = _hipotenusa(exceso_col, exceso_row)
    return (falta == 0.0), falta


# --------------------------------------------------------------------------
# Modelo del mundo
# --------------------------------------------------------------------------

class Mundo:
    """Vista del último mensaje recibido.

    La posición que publica la visión ya viene corregida: la organización
    compensa el paralaje (la cámara ve el marcador a 9 cm de altura y el cubo
    a 6, pero reporta ambos a nivel de piso) y también el desfase entre el
    marcador y el centro de giro del robot. O sea, lo que llega acá es
    directamente el centro de rotación, que es lo que necesita el control.
    """

    def __init__(self, mi_id, id_companero):
        self.mi_id = mi_id
        self.id_companero = id_companero
        self.msg = None

    def actualizar(self, msg):
        self.msg = msg

    # --- estado de la ronda ---

    def fase(self):
        return self.msg["phase"] if self.msg else "IDLE"

    def corriendo(self):
        """Solo RUNNING. Para saber si la ronda formal ya empezó."""
        return self.fase() == "RUNNING"

    def activa(self):
        """READY o RUNNING: cuándo el robot tiene que estar trabajando.

        torneo.md 4.3 y 4.4: la detección de READY debe provocar
        automáticamente el inicio de la estrategia, y ese cambio marca el
        inicio oficial del cronometraje.

        READY dura 60 segundos (preparacion_ms). Esperar a RUNNING los regala,
        y además el reglamento prohíbe arrancar con un botón (4.5 y 4.6): el
        arranque tiene que ser por telemetría.
        """
        return self.fase() in ("READY", "RUNNING")

    def restante_ms(self):
        return self.msg["clock"]["remaining_ms"] if self.msg else 0

    # --- entidades ---

    def rover(self, id_):
        """Busca por IDENTIDAD, nunca por posición en la lista.

        El orden de la lista no está garantizado y la cantidad cambia entre
        mensajes: si un rover se tapa un instante, el que estaba primero pasa
        a ser otro. Buscar por índice es el error que funciona en la compu y
        falla en la cancha.
        """
        for r in self.msg["rovers"]:
            if r["id"] == id_:
                return r
        return None

    def yo(self):
        return self.rover(self.mi_id)

    def companero(self):
        return self.rover(self.id_companero)

    def cubo(self, color):
        """El color ES la identidad: no hay dos cubos del mismo color."""
        for c in self.msg["cubes"]:
            if c["color"] == color:
                return c
        return None

    def depot(self, color):
        """Los centros de acopio son fijos, pero igual se leen del mensaje:
        viajan en cada uno y leerlos no cuesta nada."""
        for d in self.msg["depots"]:
            if d["color"] == color:
                return d
        return None

    def colores(self):
        return [c["color"] for c in self.msg["cubes"]]

    # --- derivados ---

    def entregado(self, color):
        """El veredicto del ÁRBITRO, no el nuestro.

        Desde la v3 viaja en el mensaje. Es el mismo que cuenta los cubos y
        cierra la ronda, así que no hay nada que discutirle: si él dice que
        entró, entró.

        Detalle que importa al verificar: el árbitro exige que el cubo se
        sostenga 1 segundo adentro antes de darlo por entregado. No esperen
        que esto cambie al instante. El tiempo oficial, en cambio, se toma del
        instante de ENTRADA, así que ese segundo no se paga.

        El cálculo local queda de respaldo para grabaciones viejas en v2.
        """
        cubo = self.cubo(color)
        if cubo is None:
            return False

        if "in_depot" in cubo:
            return bool(cubo["in_depot"])

        depot = self.depot(color)
        if depot is None:
            return False
        adentro, _ = cubo_en_zona(
            cubo, depot, self.msg["depot_size"], self.msg["grid"],
            self.msg["cube_side"])
        return adentro

    def falta_para_entregar(self, color):
        """Cuántas celdas le faltan al cubo. 0 = ya está adentro.

        Esto el contrato no lo publica, así que se calcula. Es el dato con el
        que se ajusta TOLERANCIA_ENTREGA cuando un cubo queda corto.
        """
        cubo = self.cubo(color)
        depot = self.depot(color)
        if cubo is None or depot is None:
            return None
        _, falta = cubo_en_zona(
            cubo, depot, self.msg["depot_size"], self.msg["grid"],
            self.msg["cube_side"])
        return falta

    def pendientes(self):
        """Cubos que todavía no están en su zona, según el árbitro."""
        return [c for c in self.colores() if not self.entregado(c)]