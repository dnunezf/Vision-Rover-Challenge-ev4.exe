"""controller.py — la secuencia de una entrega y el lazo de control.

Convierte "mi objetivo es el cubo verde" en "mové los motores así, ahora".

Sin hardware: devuelve (avance, giro) y quien los aplica es motion.py. Así
este módulo se prueba entero en la laptop.

CUÁNDO ARRANCA
--------------
En READY, no en RUNNING. El reglamento del torneo (4.3) obliga a que la
detección de READY dispare la estrategia, y el cronometraje arranca ahí
(4.4). READY dura 60 segundos: esperar a RUNNING los regala.

LOS ESTADOS DE UNA ENTREGA
    ESPERANDO   → fase IDLE o FINISHED, o el retardo de salida
    IR_APROX    → viajar al punto de aproximación
    ALINEAR     → girar en el sitio hasta apuntar a la zona
    EMPUJAR     → avanzar recto llevando el cubo
    RETROCEDER  → soltar y despejar la vista de la cámara
    VERIFICAR   → esperar el veredicto del árbitro
    LISTO       → no queda nada por hacer

Cada estado con movimiento tiene TIMEOUT. Un rover trabado sin timeout gasta
la ronda entera empujando una pared, y es la forma más común de perder.
"""

import world
import planner

# --------------------------------------------------------------------------
# Constantes  [AJUSTAR en la cancha]
# --------------------------------------------------------------------------

MI_ID = 10                    # lo pisa config.py en cada robot
TIENE_PRIORIDAD = True        # el 10 pasa, el 11 cede
RETARDO_SALIDA_MS = 0         # el 11 espera 3000

# Ganancias del lazo. Cuánto throttle por unidad de error.
KP_ANGULAR = 0.011            # por grado
KP_LINEAL = 0.09              # por celda

THROTTLE_MAX = 0.60
THROTTLE_CRUCERO = 0.45
THROTTLE_EMPUJE = 0.35        # empujando: más lento, más control
THROTTLE_GIRO = 0.30

# Si el error angular pasa esto, se gira SIN avanzar.
UMBRAL_GIRO_PURO = 40.0       # grados
TOLERANCIA_ANGULO = 6.0       # grados: se considera alineado
TOLERANCIA_POSICION = 0.5     # celdas: se considera que llegó

TOLERANCIA_ENTREGA = 1.0      # celdas: cuándo frenar el empuje
RETROCESO_CELDAS = 5.0        # cuánto retroceder para despejar

# Datos viejos
EDAD_FRESCA_MS = 250
EDAD_DUDOSA_MS = 1500

# Deconflicción
RADIO_CEDER_PASO = 5.0        # celdas = 100 mm
ANGULO_CEDER_PASO = 60.0      # solo cedo si lo tengo por delante

# Timeouts
TIMEOUT_IR_APROX_MS = 55000
TIMEOUT_ALINEAR_MS = 6000
TIMEOUT_EMPUJAR_MS = 55000

# El árbitro exige 1 segundo sostenido antes de dar un cubo por entregado, y
# el retroceso tarda. 4 segundos cubren las dos cosas con margen.
TIMEOUT_VERIFICAR_MS = 4000
MAX_REINTENTOS = 3

# Si el cubo quedó a menos de esto, el intento NO cuenta como fallido:
# estuvo a punto de entrar y otra pasada casi seguro lo mete. Sin esto, un
# cubo al que le faltaban 3 mm se postergaba igual que uno que ni se acercó.
FALTA_CASI_DENTRO = 1.0       # celdas = 20 mm

ESPERANDO = "ESPERANDO"
IR_APROX = "IR_APROX"
ALINEAR = "ALINEAR"
EMPUJAR = "EMPUJAR"
RETROCEDER = "RETROCEDER"
VERIFICAR = "VERIFICAR"
LISTO = "LISTO"


def _limitar(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


class Controlador:

    def __init__(self, mundo, ahora_ms):
        self.mundo = mundo
        self.estado = ESPERANDO
        self.objetivo = None              # color del cubo
        self.entrada_estado = ahora_ms    # cuándo entré al estado actual
        self.arranque_ronda = None
        self.reintentos = {}
        self.postergados = []
        self.pos_retroceso = None
        self.offset_cubo = None

    # ----------------------------------------------------------------------
    # Transiciones
    # ----------------------------------------------------------------------

    def _ir_a(self, estado, ahora_ms):
        if estado != self.estado:
            print("estado:", self.estado, "->", estado, "| cubo:", self.objetivo)
            self.estado = estado
            self.entrada_estado = ahora_ms
            if estado != EMPUJAR:
                self.offset_cubo = None

    def _en_estado_ms(self, ahora_ms):
        return ahora_ms - self.entrada_estado

    def _abandonar(self, ahora_ms, falta=None):
        """El intento falló. Se cuenta y se vuelve a empezar.

        Si el cubo quedó MUY cerca (falta < FALTA_CASI_DENTRO), el intento no
        se cuenta: insistir es lo correcto y postergarlo sería tirar el
        trabajo hecho. Un timeout, en cambio, siempre cuenta, porque ahí no
        sabemos cuánto faltaba.

        Tras varios fallos reales el cubo se POSTERGA, no se abandona para
        siempre: si al final es lo único que queda, hay que volver a
        intentarlo.
        """
        casi = falta is not None and falta < FALTA_CASI_DENTRO

        if self.objetivo and not casi:
            n = self.reintentos.get(self.objetivo, 0) + 1
            self.reintentos[self.objetivo] = n
            if n >= MAX_REINTENTOS:
                print("postergo", self.objetivo, "tras", n, "intentos")
                if self.objetivo not in self.postergados:
                    self.postergados.append(self.objetivo)
                self.reintentos[self.objetivo] = 0
                self.objetivo = None

        # Reiniciar el cronómetro del estado SIEMPRE, aunque ya estemos en
        # IR_APROX.
        #
        # _ir_a() no hace nada cuando el estado no cambia, así que si se
        # abandona DESDE IR_APROX (por su propio timeout), `entrada_estado`
        # quedaba intacto y el timeout seguía vencido. Al ciclo siguiente
        # volvía a vencer, volvía a abandonar, y a los tres ciclos postergaba
        # otro cubo. Para siempre, 25 veces por segundo, devolviendo (0, 0):
        # el robot congelado gastando la ronda entera sin moverse.
        #
        # Detectado el 3-oct corriendo en el robot real contra el
        # mock_publisher. El simulador no lo veía porque ahí el rover sí
        # avanza y llega al punto antes de que el timeout se venza.
        if self.estado == IR_APROX:
            self.entrada_estado = ahora_ms
        else:
            self._ir_a(IR_APROX, ahora_ms)

    # ----------------------------------------------------------------------
    # Lazo de control
    # ----------------------------------------------------------------------

    def _hacia_punto(self, col_destino, row_destino, throttle):
        """Go-to-point en lazo cerrado. Devuelve (avance, giro, llegué).

        Control proporcional: la corrección es proporcional al error. Mucho
        error, mucha corrección; poco error, poca. Así el robot desacelera
        solo al acercarse, en vez de frenar de golpe.
        """
        yo = self.mundo.yo()
        if yo is None:
            return 0.0, 0.0, False

        d = world.distancia(yo["col"], yo["row"], col_destino, row_destino)
        if d < TOLERANCIA_POSICION:
            return 0.0, 0.0, True

        rumbo = world.rumbo_hacia(yo["col"], yo["row"], col_destino, row_destino)

        # normalizar es lo que hace que gire por el camino corto: sin esto,
        # un error de -10 grados se leería como 350 y daría la vuelta larga.
        error = world.normalizar(rumbo - yo["theta"])

        w = _limitar(KP_ANGULAR * error, -THROTTLE_GIRO, THROTTLE_GIRO)

        # Si estoy muy desalineado, giro SIN avanzar. Si avanzara, el robot
        # describiría una curva larga y se llevaría el cubo puesto.
        if abs(error) > UMBRAL_GIRO_PURO:
            return 0.0, w, False

        v = _limitar(KP_LINEAL * d, 0.0, throttle)
        return v, w, False

    def _girar_a(self, rumbo_deseado):
        """Gira en el sitio. Devuelve (giro, alineado)."""
        yo = self.mundo.yo()
        if yo is None:
            return 0.0, False
        error = world.normalizar(rumbo_deseado - yo["theta"])
        if abs(error) < TOLERANCIA_ANGULO:
            return 0.0, True
        return _limitar(KP_ANGULAR * error, -THROTTLE_GIRO, THROTTLE_GIRO), False

    # ----------------------------------------------------------------------
    # Deconflicción
    # ----------------------------------------------------------------------

    def _debo_ceder(self):
        """Prioridad FIJA: el rover con prioridad nunca cede.

        Si los dos cedieran, se quedarían trabados mirándose para siempre,
        como dos personas en un pasillo. Con prioridad fija eso no puede
        pasar: uno siempre avanza.

        Se resuelve con la pura telemetría, sin comunicación entre rovers.
        """
        if TIENE_PRIORIDAD:
            return False

        yo = self.mundo.yo()
        otro = self.mundo.companero()
        if yo is None or otro is None:
            return False

        # No freno por un dato viejo: si hace rato que no lo veo, esa
        # posición ya no significa nada.
        if otro.get("age_ms", 0) > EDAD_DUDOSA_MS:
            return False

        d = world.distancia(yo["col"], yo["row"], otro["col"], otro["row"])
        if d > RADIO_CEDER_PASO:
            return False

        # Solo cedo si lo tengo por DELANTE. Si está atrás, no me estorba.
        rumbo = world.rumbo_hacia(yo["col"], yo["row"], otro["col"], otro["row"])
        return abs(world.normalizar(rumbo - yo["theta"])) < ANGULO_CEDER_PASO

    # ----------------------------------------------------------------------
    # Un ciclo
    # ----------------------------------------------------------------------

    def paso(self, ahora_ms):
        """Se llama una vez por mensaje. Devuelve (avance, giro)."""
        m = self.mundo

        # READY cuenta como activa: el cronómetro oficial ya corre y el
        # reglamento obliga a arrancar ahí (torneo.md 4.3 y 4.4).
        if not m.activa():
            self.estado = ESPERANDO
            self.arranque_ronda = None
            return 0.0, 0.0

        if self.arranque_ronda is None:
            self.arranque_ronda = ahora_ms

        # Salida escalonada: los dos rovers salen del MISMO punto, y ese es
        # el choque más probable de toda la ronda. Se cuenta desde READY.
        if ahora_ms - self.arranque_ronda < RETARDO_SALIDA_MS:
            return 0.0, 0.0

        if m.yo() is None:
            return 0.0, 0.0        # este cuadro no me vio

        if self._debo_ceder():
            return 0.0, 0.0

        # --- elegir objetivo ---
        if self.estado in (ESPERANDO, IR_APROX):
            self.objetivo = planner.mi_objetivo(m, self.objetivo,
                                                self.postergados)
            if self.objetivo is None:
                if self.estado != LISTO:
                    self._ir_a(LISTO, ahora_ms)
                return 0.0, 0.0
            if self.estado == ESPERANDO:
                self._ir_a(IR_APROX, ahora_ms)

        if self.estado == LISTO:
            # Puede reaparecer trabajo: un cubo sacado de su zona sin querer,
            # o el compañero que se quedó sin batería.
            if planner.mi_objetivo(m, None, self.postergados) is not None:
                self._ir_a(IR_APROX, ahora_ms)
            return 0.0, 0.0

        cubo = m.cubo(self.objetivo) if self.objetivo else None
        depot = m.depot(self.objetivo) if self.objetivo else None
        if cubo is None or depot is None:
            self._ir_a(IR_APROX, ahora_ms)
            return 0.0, 0.0

        # ------------------------------------------------------------------
        if self.estado == IR_APROX:
            if self._en_estado_ms(ahora_ms) > TIMEOUT_IR_APROX_MS:
                self._abandonar(ahora_ms)
                return 0.0, 0.0

            col, row, _ = planner.punto_aproximacion(cubo, depot, m.msg["grid"])
            v, w, llegue = self._hacia_punto(col, row, THROTTLE_CRUCERO)
            if llegue:
                self._ir_a(ALINEAR, ahora_ms)
            return v, w

        # ------------------------------------------------------------------
        if self.estado == ALINEAR:
            if self._en_estado_ms(ahora_ms) > TIMEOUT_ALINEAR_MS:
                self._abandonar(ahora_ms)
                return 0.0, 0.0

            _, _, rumbo = planner.punto_aproximacion(cubo, depot, m.msg["grid"])
            w, alineado = self._girar_a(rumbo)
            if alineado:
                self._ir_a(EMPUJAR, ahora_ms)
            return 0.0, w

        # ------------------------------------------------------------------
        if self.estado == EMPUJAR:
            if self._en_estado_ms(ahora_ms) > TIMEOUT_EMPUJAR_MS:
                self._abandonar(ahora_ms)
                return 0.0, 0.0

            yo = m.yo()

            # Al enganchar, anoto a qué distancia tengo el cubo. Si después
            # la cámara lo pierde, uso esa distancia para estimar dónde está
            # a partir de mi propia pose.
            if self.offset_cubo is None:
                if cubo["age_ms"] < EDAD_DUDOSA_MS:
                    self.offset_cubo = world.distancia(
                        yo["col"], yo["row"], cubo["col"], cubo["row"])
                else:
                    self.offset_cubo = planner.DISTANCIA_APROXIMACION

            if cubo["age_ms"] < EDAD_FRESCA_MS:
                est_col, est_row = cubo["col"], cubo["row"]     # dato real
            else:
                est_col, est_row = world.adelante(
                    yo["col"], yo["row"], yo["theta"], self.offset_cubo)

            # Si el árbitro ya lo dio por entregado, dejar de empujar YA.
            # Seguir empujando un cubo que ya cuenta solo puede sacarlo, y
            # sacarlo borra su tiempo (torneo.md 7.4 y 8.8).
            if m.entregado(self.objetivo):
                self.pos_retroceso = (yo["col"], yo["row"])
                self._ir_a(RETROCEDER, ahora_ms)
                return 0.0, 0.0

            # Freno cuando el CUBO llegó al centro de la zona, no cuando
            # llegué yo. Pasarme de la línea del borde hace que no cuente.
            if world.distancia(est_col, est_row, depot["col"], depot["row"]) < TOLERANCIA_ENTREGA:
                self.pos_retroceso = (yo["col"], yo["row"])
                self._ir_a(RETROCEDER, ahora_ms)
                return 0.0, 0.0

            v, w, _ = self._hacia_punto(depot["col"], depot["row"], THROTTLE_EMPUJE)
            return v, w

        # ------------------------------------------------------------------
        if self.estado == RETROCEDER:
            # Apartarse NO es cortesía: el árbitro necesita ver el cubo en su
            # posición nueva para contarlo. Un cubo que YA estaba adentro y
            # queda tapado sigue contando, pero uno tapado durante todo el
            # empuje nunca se vio entrar.
            yo = m.yo()
            origen = self.pos_retroceso or (yo["col"], yo["row"])
            if world.distancia(yo["col"], yo["row"], origen[0], origen[1]) >= RETROCESO_CELDAS:
                self._ir_a(VERIFICAR, ahora_ms)
                return 0.0, 0.0
            return -THROTTLE_CRUCERO, 0.0

        # ------------------------------------------------------------------
        if self.estado == VERIFICAR:
            # El veredicto es del árbitro (in_depot), y tarda 1 segundo
            # sostenido en ponerse en true. Por eso no se juzga al instante:
            # se espera a que el dato esté fresco y se le cree a él.
            cubo_actual = m.cubo(self.objetivo)
            fresco = cubo_actual and cubo_actual["age_ms"] < EDAD_FRESCA_MS

            if fresco and m.entregado(self.objetivo):
                print("ENTREGADO", self.objetivo,
                      "| restan", m.restante_ms() // 1000, "s")
                self.objetivo = None
                self._ir_a(IR_APROX, ahora_ms)
                return 0.0, 0.0

            # Todavía no: se le da tiempo a la permanencia antes de declarar
            # el fallo. Solo al vencer el timeout se cuenta como intento.
            if self._en_estado_ms(ahora_ms) > TIMEOUT_VERIFICAR_MS:
                falta = m.falta_para_entregar(self.objetivo)
                if falta is None:
                    print("no entró:", self.objetivo, "(sin dato)")
                    self._abandonar(ahora_ms)
                else:
                    print("no entró:", self.objetivo, "falta", round(falta, 2))
                    self._abandonar(ahora_ms, falta)
            return 0.0, 0.0

        return 0.0, 0.0