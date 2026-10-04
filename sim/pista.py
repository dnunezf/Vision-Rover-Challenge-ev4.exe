"""sim/pista.py — simulador de lazo cerrado.

Corre en la laptop, con Python normal.

POR QUÉ EXISTE
    El mock_publisher.py de CENFOTEC mueve los rovers al azar e IGNORA lo que
    hagamos. Sirve para probar el parseo, no la navegación: si nuestro código
    dice "girá", el rover simulado no gira.

    Este simulador cierra el lazo: los comandos mueven el robot, el robot
    empuja los cubos, y de ahí sale la telemetría del ciclo siguiente.

    Importa los MISMOS módulos que corren en el rover (world, planner,
    controller), así que lo que se valida acá es el código de verdad, no una
    maqueta parecida.

USO
    py sim\\pista.py                    una corrida, con detalle
    py sim\\pista.py --n 30             30 corridas, estadísticas
    py sim\\pista.py --n 30 --dificultad 0.8

LA MÉTRICA que importa no es "funcionó una vez": es en cuántas de 30 corridas
se entregan los tres cubos, y en qué tiempo.
"""

import argparse
import json
import math
import os
import random
import sys

# Para poder importar los módulos del firmware desde acá.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "firmware"))

import world          # noqa: E402
import planner        # noqa: E402
import controller     # noqa: E402

DT = 0.05             # 20 Hz, igual que la telemetría real

# Velocidad del robot a throttle 1.0.  [MEDIDO 3-oct-2026]
# Sale de 27 cm en 5 s a throttle 0.45 (= 2.7 celdas/s) y de 90 grados en
# 0.8 s a throttle 0.30, extrapolado a fondo.
CELDAS_POR_S_A_FONDO = 6.0
GRADOS_POR_S_A_FONDO = 375.0

RADIO_EMPUJE = 1.8        # celdas: a menos de esto, el rover arrastra el cubo
RADIO_OCLUSION = 2.0      # celdas: a menos de esto, la cámara no ve el cubo
RUIDO_POS = 0.06          # celdas, parecido al del simulador oficial
RUIDO_THETA = 1.5         # grados
PROB_PERDIDA_ROVER = 0.02 # 2% de los cuadros no se ve un rover

# --- CAMBIO 1: el veredicto del árbitro ----------------------------------
# El árbitro NO usa nuestra cuenta conservadora: agranda la ventana 2.5 mm
# por lado, porque el margen de media diagonal supone el peor giro del cubo
# y eso es más estricto de lo necesario. Y exige 1 segundo sostenido antes
# de dar un cubo por entregado, para que la cuenta no titile en el borde.
#
# Los dos valores salen de contrato/config_simulador.json de CENFOTEC:
#   arbitro.tolerancia_mm         = 2.5
#   arbitro.permanencia_minima_ms = 1000
TOLERANCIA_ARBITRO = 0.125      # celdas (2.5 mm / 20 mm por celda)
PERMANENCIA_MS = 1000


class RoverSim:
    """Un rover en el mundo simulado.

    Tiene DOS posiciones: la real (col, row, theta) y la reportada (rep).
    Esa separación es la clave del simulador: el código del robot solo ve la
    reportada, con su ruido y sus oclusiones, igual que en la cancha.
    """
    def __init__(self, id_, col, row, theta):
        self.id = id_
        self.col, self.row, self.theta = col, row, theta
        self.rep = (col, row, theta)
        self.visto_ms = 0


class CuboSim:
    def __init__(self, color, col, row):
        self.color, self.col, self.row = color, col, row
        self.rep = (col, row)
        self.visto_ms = 0


class Pista:

    def __init__(self, escenario, semilla=None):
        self.rng = random.Random(semilla)
        self.t_ms = 0
        self.grid = escenario["grid"]
        self.start = escenario["start"]
        self.depots = escenario["depots"]
        self.depot_size = escenario["depot_size"]
        self.cube_side = escenario["cube_side"]
        self.rovers = [RoverSim(r["id"], r["col"], r["row"], r["theta"])
                       for r in escenario["rovers"]]
        self.cubos = [CuboSim(c["color"], c["col"], c["row"])
                      for c in escenario["cubes"]]
        self.seq = 0

        # --- CAMBIO 2: memoria de la permanencia ---
        # El árbitro es la única pieza con memoria entre cuadros: necesita
        # saber desde cuándo está adentro cada cubo. Se borra apenas sale,
        # porque lo que se mide es que SE QUEDE.
        self._adentro_desde = {}      # color -> ts_ms de cuando entró

    # ------------------------------------------------------------------
    # Física
    # ------------------------------------------------------------------

    def mover(self, rover, izq, der):
        """Modelo diferencial de dos ruedas.

        La media de las dos ruedas da el avance; la diferencia, el giro. Es
        toda la física que necesita un robot de tracción diferencial.
        """
        v = (izq + der) / 2.0 * CELDAS_POR_S_A_FONDO
        w = (der - izq) / 2.0 * GRADOS_POR_S_A_FONDO

        rover.theta = (rover.theta + w * DT) % 360.0

        rad = math.radians(rover.theta)
        dcol = math.cos(rad) * v * DT
        drow = -math.sin(rad) * v * DT      # el mismo menos de siempre

        ncol, nrow = rover.col + dcol, rover.row + drow
        if 0 <= ncol <= self.grid["cols"] and 0 <= nrow <= self.grid["rows"]:
            rover.col, rover.row = ncol, nrow
            self._empujar(rover, dcol, drow)

    def _empujar(self, rover, dcol, drow):
        """El rover arrastra el cubo SOLO si avanza hacia él.

        Los brazos en V empujan, no agarran: un rover que retrocede suelta
        el cubo. Sin esta condición el rover se lleva el cubo de vuelta al
        apartarse, y ninguna entrega queda firme.
        """
        for c in self.cubos:
            dx = c.col - rover.col
            dy = c.row - rover.row
            if math.hypot(dx, dy) >= RADIO_EMPUJE:
                continue
            # Producto punto: positivo = el rover se mueve HACIA el cubo.
            if dcol * dx + drow * dy <= 0:
                continue
            ncol, nrow = c.col + dcol * 0.9, c.row + drow * 0.9
            if 0 <= ncol <= self.grid["cols"] and 0 <= nrow <= self.grid["rows"]:
                c.col, c.row = ncol, nrow

    # ------------------------------------------------------------------
    # El árbitro
    # ------------------------------------------------------------------

    # --- CAMBIO 3: la cuenta del árbitro ---
    def _veredicto_arbitro(self, color, col, row):
        """¿El árbitro daría este cubo por dentro de su zona?

        Es la cuenta conservadora de world.cubo_en_zona MÁS la tolerancia
        del árbitro. La diferencia importa: con la zona de 200 x 150 mm y el
        cubo de 60, la ventana conservadora mide 115.2 x 65.1 mm y la del
        árbitro 120.1 x 70.1. Hay cubos que nuestra cuenta rechaza y el juez
        acepta, y publicarlo acá es lo que hace que el simulador deje de ser
        más duro que el torneo.
        """
        depot = next(d for d in self.depots if d["color"] == color)
        lado = world.lado_mas_cercano(
            depot["col"], depot["row"], self.grid["cols"], self.grid["rows"])

        if lado in ("arriba", "abajo"):
            semi_col = self.depot_size["length"] / 2.0
            semi_row = self.depot_size["depth"] / 2.0
        else:
            semi_col = self.depot_size["depth"] / 2.0
            semi_row = self.depot_size["length"] / 2.0

        margen = self.cube_side * math.sqrt(2.0) / 2.0 - TOLERANCIA_ARBITRO

        return (abs(col - depot["col"]) <= semi_col - margen and
                abs(row - depot["row"]) <= semi_row - margen)

    # ------------------------------------------------------------------
    # Telemetría
    # ------------------------------------------------------------------

    def telemetria(self, fase="RUNNING", total_ms=600000):
        """Arma el mensaje igual que el contrato v3, con ruido y oclusiones.

        Acá está lo que hace útil al simulador: no publica la verdad, publica
        una versión degradada de la verdad, como haría la cámara.
        """
        self.seq += 1

        rovers = []
        for r in self.rovers:
            if self.rng.random() >= PROB_PERDIDA_ROVER:
                # Se ve: actualizo lo reportado, con ruido.
                r.rep = (r.col + self.rng.gauss(0, RUIDO_POS),
                         r.row + self.rng.gauss(0, RUIDO_POS),
                         (r.theta + self.rng.gauss(0, RUIDO_THETA)) % 360.0)
                r.visto_ms = self.t_ms
            # Si no se ve, rep queda como estaba y age_ms crece solo.
            rovers.append({"id": r.id, "col": round(r.rep[0], 3),
                           "row": round(r.rep[1], 3),
                           "theta": round(r.rep[2], 2),
                           "age_ms": self.t_ms - r.visto_ms})

        # --- CAMBIO 4: cada cubo viaja con el veredicto ---
        cubes = []
        for c in self.cubos:
            tapado = any(math.hypot(r.col - c.col, r.row - c.row) < RADIO_OCLUSION
                         for r in self.rovers)
            if not tapado:
                c.rep = (c.col + self.rng.gauss(0, RUIDO_POS),
                         c.row + self.rng.gauss(0, RUIDO_POS))
                c.visto_ms = self.t_ms

            # El veredicto va sobre la posición REPORTADA, igual que el
            # árbitro: él juzga lo que ve, no la verdad. Un cubo tapado sigue
            # contando con su última posición buena, y eso es deliberado: si
            # no, la cuenta se caería justo en el momento de la entrega, que
            # es cuando hay un rover encima.
            adentro = self._veredicto_arbitro(c.color, c.rep[0], c.rep[1])
            if adentro:
                desde = self._adentro_desde.setdefault(c.color, self.t_ms)
                contado = (self.t_ms - desde) >= PERMANENCIA_MS
            else:
                # Salió, por el motivo que sea: la permanencia se cuenta de
                # cero si vuelve a entrar. Demora ENTRAR, nunca SALIR.
                self._adentro_desde.pop(c.color, None)
                contado = False

            cubes.append({"color": c.color, "col": round(c.rep[0], 3),
                          "row": round(c.rep[1], 3),
                          "age_ms": self.t_ms - c.visto_ms,
                          "in_depot": contado})

        return {"v": 3, "seq": self.seq, "ts_ms": self.t_ms, "phase": fase,
                "clock": {"elapsed_ms": self.t_ms,
                          "remaining_ms": max(0, total_ms - self.t_ms),
                          "total_ms": total_ms},
                "grid": self.grid, "rovers": rovers, "cubes": cubes,
                "obstacles": [], "start": self.start, "depots": self.depots,
                "depot_size": self.depot_size, "cube_side": self.cube_side}

    def verdad_entregados(self):
        """Cuántos cubos contaría el árbitro, sobre la posición REAL.

        Se usa la ventana del ÁRBITRO y no la conservadora, porque lo que
        queremos medir es cuántos puntos daría el torneo, no cuántos daría
        nuestra cuenta interna. Sobre la posición real y no la reportada
        para que el puntaje no dependa del ruido de la cámara.
        """
        return sum(1 for c in self.cubos
                   if self._veredicto_arbitro(c.color, c.col, c.row))


# ----------------------------------------------------------------------
# Escenarios
# ----------------------------------------------------------------------

ESCENARIO_BASE = {
    "grid": {"cols": 43, "rows": 43, "cell_mm": 20.0},
    "start": {"col": 3.75, "row": 21.5},
    "depots": [{"color": "green", "col": 21.5, "row": 3.75},
               {"color": "red", "col": 39.25, "row": 21.5},
               {"color": "blue", "col": 21.5, "row": 39.25}],
    "depot_size": {"length": 10.0, "depth": 7.5},
    "cube_side": 3.0,
    "rovers": [{"id": 10, "col": 4.0, "row": 17.5, "theta": 0.0},
               {"id": 11, "col": 4.0, "row": 25.5, "theta": 0.0}],
    "cubes": [{"color": "green", "col": 26.0, "row": 10.0},
              {"color": "blue", "col": 15.0, "row": 29.0},
              {"color": "red", "col": 33.0, "row": 26.0}],
}


def escenario_aleatorio(rng, dificultad=0.2):
    """Aproxima el generador oficial: distancia media = 6.5 + 35 × D.

    Con dificultad 0.2 los cubos quedan a ~13 celdas de su zona; con 0.8, a
    ~34. Es la misma fórmula que usa CENFOTEC.
    """
    esc = json.loads(json.dumps(ESCENARIO_BASE))
    objetivo = 6.5 + 35.0 * dificultad
    for cubo in esc["cubes"]:
        depot = next(d for d in esc["depots"] if d["color"] == cubo["color"])
        for _ in range(200):
            col = rng.uniform(5, 38)
            row = rng.uniform(5, 38)
            manhattan = abs(col - depot["col"]) + abs(row - depot["row"])
            if abs(manhattan - objetivo) < 4.0:
                cubo["col"], cubo["row"] = round(col, 2), round(row, 2)
                break
    return esc


# ----------------------------------------------------------------------
# Una ronda
# ----------------------------------------------------------------------

def correr(escenario, semilla=None, verbose=True, limite_ms=600000):
    """Una ronda completa. Devuelve (cubos_entregados, ms_del_ultimo)."""
    pista = Pista(escenario, semilla)

    mundos, ctrls = {}, {}
    for r in pista.rovers:
        otro = [x.id for x in pista.rovers if x.id != r.id][0]
        m = world.Mundo(r.id, otro)
        mundos[r.id] = m
        ctrls[r.id] = controller.Controlador(m, 0)

    entregados_previos = 0
    ultimo_ms = 0

    while pista.t_ms < limite_ms:
        msg = pista.telemetria()

        for r in pista.rovers:
            # Cada rover corre con su propio MI_ID. En el robot real eso lo
            # fija config.py; acá se conmuta para simular los dos.
            controller.MI_ID = r.id
            controller.TIENE_PRIORIDAD = (r.id == 10)
            controller.RETARDO_SALIDA_MS = 0 if r.id == 10 else 3000

            mundos[r.id].actualizar(msg)
            v, w = ctrls[r.id].paso(pista.t_ms)

            # El modelo diferencial al revés: de (avance, giro) a dos ruedas.
            izq = max(-1.0, min(1.0, v - w))
            der = max(-1.0, min(1.0, v + w))
            pista.mover(r, izq, der)

        n = pista.verdad_entregados()
        if n > entregados_previos:
            ultimo_ms = pista.t_ms
            if verbose:
                print("  cubo #{} a los {:.1f} s".format(n, pista.t_ms / 1000.0))
        entregados_previos = n

        if n == len(pista.cubos):
            break

        pista.t_ms += int(DT * 1000)

    return entregados_previos, ultimo_ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--dificultad", type=float, default=0.2)
    ap.add_argument("--semilla", type=int, default=2026)
    args = ap.parse_args()

    rng = random.Random(args.semilla)
    resultados = []

    for i in range(args.n):
        esc = ESCENARIO_BASE if args.n == 1 else escenario_aleatorio(rng, args.dificultad)
        n, ms = correr(esc, semilla=rng.randint(0, 10**6), verbose=(args.n == 1))
        resultados.append((n, ms))
        if args.n > 1:
            print("corrida {:>3}: {} cubos, último a {:.1f} s".format(
                i + 1, n, ms / 1000.0))

    completas = sorted(ms for n, ms in resultados if n == 3)
    print("\n=== {} corridas, dificultad {} ===".format(args.n, args.dificultad))
    print("3 cubos: {}/{}  ({:.0f}%)".format(
        len(completas), len(resultados),
        100.0 * len(completas) / len(resultados)))
    print("promedio de cubos: {:.2f}".format(
        sum(n for n, _ in resultados) / len(resultados)))
    if completas:
        print("tiempo: mejor {:.1f}s  mediana {:.1f}s  peor {:.1f}s".format(
            completas[0] / 1000.0,
            completas[len(completas) // 2] / 1000.0,
            completas[-1] / 1000.0))


if __name__ == "__main__":
    main()