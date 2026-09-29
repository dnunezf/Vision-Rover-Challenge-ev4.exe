"""world.py — geometría y modelo del mundo.

Traduce el JSON crudo de la visión en respuestas útiles.

No importa nada de hardware a propósito. Así este mismo archivo corre en el
ESP32 y en la laptop, y se puede probar con py sin tener el robot enfrente.

Regla: el resto del código NUNCA toca el diccionario del mensaje. Todo pasa
por acá. Si mañana el contrato cambia un nombre de campo, se arregla en un
solo lugar.
"""

import math


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
    return math.hypot(col_b - col_a, row_b - row_a)


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

    Es la MISMA cuenta que corre el juez. No inventar otra versión: si
    difieren, un cubo puede estar adentro para nosotros y afuera para la
    organización, y eso se descubre el día del torneo.

    La sutileza está en el margen. El mensaje dice dónde está el centro del
    cubo, pero NO cómo está girado: la organización decidió no publicar el
    ángulo porque para atrapar el cubo da igual, y un marcador encima taparía
    el color. Para que el veredicto valga en cualquier rotación se usa el peor
    caso: media diagonal del cubo, que es lado × raíz(2) / 2.

    Con cubos de 3 celdas eso da 2.12 celdas = 42.4 mm. Por eso el cubo tiene
    que quedar CENTRADO en la zona y no empujado contra el borde exterior.
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

    falta = math.hypot(exceso_col, exceso_row)
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
        return self.fase() == "RUNNING"

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
        cubo = self.cubo(color)
        depot = self.depot(color)
        if cubo is None or depot is None:
            return False
        adentro, _ = cubo_en_zona(
            cubo, depot, self.msg["depot_size"], self.msg["grid"],
            self.msg["cube_side"])
        return adentro

    def falta_para_entregar(self, color):
        """Cuántas celdas le faltan al cubo. 0 = ya está adentro."""
        cubo = self.cubo(color)
        depot = self.depot(color)
        if cubo is None or depot is None:
            return None
        _, falta = cubo_en_zona(
            cubo, depot, self.msg["depot_size"], self.msg["grid"],
            self.msg["cube_side"])
        return falta

    def pendientes(self):
        """Cubos que todavía no están en su zona."""
        return [c for c in self.colores() if not self.entregado(c)]