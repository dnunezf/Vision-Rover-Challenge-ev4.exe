"""code.py — el programa del robot. CircuitPython lo corre solo al arrancar.

Junta todas las piezas: recibe telemetría, decide, mueve los motores.

    vision_client  →  world  →  planner  →  controller  →  motion  →  motores

Este archivo no tiene lógica propia: es el cableado. Toda la inteligencia
está en los módulos, que son los mismos que se prueban en la laptop con el
simulador.

EL ARRANQUE: SIN BOTÓN
----------------------
El reglamento del torneo lo prohíbe. 4.5 y 4.6: no se permite intervención
humana ni apretar botones para iniciar la estrategia después del cambio a
READY. 4.7: la detección de READY y el inicio deben formar parte del software
del equipo.

Así que este archivo arranca al encender la placa, se conecta, y queda
girando. Quien decide cuándo moverse es el controlador, mirando la fase de la
telemetría:

    IDLE      encendido y conectado, quieto. Acá se colocan los robots.
    READY     1 minuto pensando, QUIETO (reglamento 9.3). Se reparte el
              trabajo pero no se mueve nada.
    RUNNING   sale solo. Acá arranca el cronómetro oficial (9.6).
    FINISHED  se detiene (10.6).

EL LED: UN COLOR POR ESTADO
---------------------------
En la cancha el robot va SIN CABLE, así que no hay consola. El LED es toda la
información disponible, y por eso lleva un color por estado y no solo tres:

    ROJO       PROBLEMA: sin telemetría fresca (watchdog) o error en el
               código. Está ciego y con los motores frenados.
    AMARILLO   conectado, en IDLE o FINISHED
    NARANJA    en READY: recibiendo y planificando, quieto a propósito
    VERDE      yendo al punto de aproximación
    AZUL       alineándose
    CELESTE    empujando el cubo
    MAGENTA    retrocediendo o esperando el veredicto del árbitro
    BLANCO     un segundo o dos: maniobra de escape (APARTARSE, DESATASCAR)
               fijo al final de la ronda: terminó, no le queda nada (LISTO)

El ROJO queda SOLO para problemas. Hasta el 4-oct también lo usaba APARTARSE,
y entonces "se puso rojo" no distinguía "se está apartando, todo bien" de
"está ciego y frenado", que es justo la pregunta que había que contestar
mirando la luz.

Con tres colores, un robot quieto en verde podía ser tres cosas distintas:
trabado alineándose, sin llegar al punto, o esperando el veredicto. Con uno por
estado se sabe desde el otro lado de la mesa.

EL WATCHDOG
-----------
Si deja de llegar telemetría, los motores van a CERO. Un robot ciego que
sigue avanzando se sale de la cancha o embiste al compañero; uno que frena
solo pierde unos segundos y se recupera cuando vuelve la señal.
"""

import gc
import time

from ideaboard import IdeaBoard

import config
import controller
import motion
import vision_client
import world

# --------------------------------------------------------------------------
# Constantes del lazo
# --------------------------------------------------------------------------

# Si no llega telemetría por más de esto, frenar. La visión publica a 20 Hz
# (un mensaje cada 50 ms), así que medio segundo son diez mensajes perdidos:
# algo anda mal.
WATCHDOG_MS = 500

# Cada cuánto imprimir el estado. No en cada vuelta: los print() por USB son
# lentos y bajarían los Hz del lazo.
PERIODO_LOG_MS = 1000

ROJO = (64, 0, 0)
NARANJA = (64, 20, 0)
AMARILLO = (64, 64, 0)
VERDE = (0, 64, 0)
AZUL = (0, 0, 64)
CELESTE = (0, 48, 48)
MAGENTA = (48, 0, 48)
BLANCO = (48, 48, 48)


# Un color por estado del controller. Se arma después de importarlo para usar
# sus constantes y no cadenas sueltas: si mañana se renombra un estado, esto
# deja de compilar en vez de quedarse mostrando el color de otro.
COLOR_DE_ESTADO = {
    controller.ESPERANDO:  AMARILLO,
    controller.APARTARSE:  BLANCO,    # maniobra: despegándose del compañero
    controller.DESATASCAR: BLANCO,    # maniobra: despegándose de un atasco
    controller.IR_APROX:   VERDE,
    controller.ALINEAR:    AZUL,
    controller.EMPUJAR:    CELESTE,
    controller.RETROCEDER: MAGENTA,
    controller.VERIFICAR:  MAGENTA,
    controller.LISTO:      BLANCO,
}


def ahora_ms():
    return time.monotonic_ns() // 1000000


# --------------------------------------------------------------------------
# Arranque
# --------------------------------------------------------------------------

ib = IdeaBoard()
ib.pixel = ROJO

print()
print("=" * 40)
print("ROVER", config.MI_ID)
print("=" * 40)

# Las constantes que distinguen a un rover del otro se copian desde config.
controller.MI_ID = config.MI_ID
controller.TIENE_PRIORIDAD = config.TIENE_PRIORIDAD
controller.RETARDO_SALIDA_MS = config.RETARDO_SALIDA_MS

motores = motion.Motores(ib)
motores.parar()

cliente = vision_client.ClienteVision(
    config.WIFI_SSID, config.WIFI_PASSWORD,
    config.VISION_HOST, config.VISION_PUERTO)
cliente.conectar_wifi()
cliente.conectar()

mundo = world.Mundo(config.MI_ID, config.ID_COMPANERO)
ctrl = controller.Controlador(mundo, ahora_ms())

ib.pixel = AMARILLO

print()
print("CONECTADO. En READY planifica quieto; arranca solo en RUNNING.")


# --------------------------------------------------------------------------
# Lazo principal
# --------------------------------------------------------------------------

t_ultimo_log = 0
vueltas = 0
t_inicio = ahora_ms()
color_actual = AMARILLO

def memoria_libre():
    """Bytes libres. CircuitPython lo tiene; Python normal no (en la laptop)."""
    return gc.mem_free() if hasattr(gc, "mem_free") else -1


# EL LAZO NO PUEDE MORIRSE
# ------------------------
# [MEDIDO 5-oct-2026, ronda de las 10:22] El robot 10 dejó de leer telemetría
# en pleno IDLE y se quedó así toda la ronda: la cámara lo veía perfecto, el
# sistema de visión descartaba el 100% de sus mensajes, y nunca se desconectó
# ni se reconectó. Es lo que pasa cuando el programa TERMINA: la placa sigue
# prendida, el socket queda abierto y nadie lo lee. El domingo a las 12:47 el
# mismo robot hizo lo mismo.
#
# La lectura de telemetría estaba fuera de todo try. Cualquier error ahí
# —y en la placa del robot 10, que ya dio MemoryError al subirle archivos,
# el candidato obvio es quedarse sin memoria— subía hasta acá y terminaba el
# programa. Un robot que se muere en IDLE pierde la ronda entera sin moverse.
#
# Ahora todo el cuerpo del lazo está protegido. Ante un error: motores a cero,
# luz roja, limpiar memoria, y seguir. Si el error era pasajero, el robot se
# recupera solo en la vuelta siguiente.
while True:
    try:
        t = ahora_ms()
        vueltas += 1

        # 1. RECIBIR. Devuelve el mensaje más nuevo, o None si no llegó nada.
        msg = cliente.ultimo()
        if msg is not None:
            mundo.actualizar(msg)

        # 2. WATCHDOG. Sin datos frescos, no se mueve. Va ANTES de decidir:
        #    ninguna lógica sirve con información vieja.
        if cliente.edad_ms() > WATCHDOG_MS:
            motores.parar()
            if color_actual != ROJO:
                ib.pixel = ROJO
                color_actual = ROJO
            time.sleep(0.02)
            continue

        # 3. DECIDIR, solo cuando llegó un mensaje NUEVO.
        #
        #    El lazo da varias vueltas por cada mensaje (20 por segundo), y
        #    antes el controlador recalculaba lo mismo en cada una: el reparto
        #    entero, con todas sus listas, dos o tres veces por mensaje. En
        #    una placa justa de memoria eso es basura de más. Decidiendo una
        #    vez por mensaje el robot hace exactamente lo que hace en el
        #    simulador, que también decide una vez por mensaje.
        if msg is not None:
            try:
                v, w = ctrl.paso(t)
            except MemoryError:
                raise
            except Exception as e:
                # Un error en la lógica no puede dejar los motores encendidos.
                print("ERROR en controller:", e)
                motores.parar()
                ib.pixel = ROJO
                color_actual = ROJO
                time.sleep(0.1)
                continue

            # 4. MOVER.
            motores.aplicar(v, w)

        # 4b. EL LED. Solo se escribe cuando CAMBIA: escribirlo en cada vuelta
        #     cuesta tiempo del lazo para mostrar lo mismo.
        # NARANJA = en READY, planificando quieto. Sirve para ver de un vistazo
        # que el robot SÍ está recibiendo telemetría durante ese minuto, aunque
        # no se mueva: amarillo e inmóvil no distingue "esperando" de "colgado".
        if mundo.puede_moverse():
            color = COLOR_DE_ESTADO.get(ctrl.estado, VERDE)
        elif mundo.puede_planificar():
            color = NARANJA
        else:
            color = AMARILLO
        if color != color_actual:
            ib.pixel = color
            color_actual = color

        # 5. LOG, una vez por segundo. De paso se ordena la memoria: una vez
        #    por segundo cuesta poco, y la deja ordenada antes de que haga
        #    falta. La memoria libre va en el log para verla en la consola.
        if t - t_ultimo_log > PERIODO_LOG_MS:
            gc.collect()
            hz = vueltas * 1000.0 / max(1, t - t_inicio)
            yo = mundo.yo()
            pos = "({:.1f},{:.1f})".format(yo["col"], yo["row"]) if yo else "??"
            print("{:5.1f}Hz | {} | {} | {} | obj={} | edad={}ms | mem={}".format(
                hz, mundo.fase(), ctrl.estado, pos, ctrl.objetivo,
                cliente.edad_ms(), memoria_libre()))
            t_ultimo_log = t

        # 6. Respirar. El lazo no tiene que ir más rápido que la telemetría.
        time.sleep(0.01)

    except MemoryError:
        motores.parar()
        gc.collect()
        print("SIN MEMORIA en el lazo, sigo. Libre ahora:", memoria_libre())
        if color_actual != ROJO:
            ib.pixel = ROJO
            color_actual = ROJO
        time.sleep(0.05)
    except Exception as e:
        motores.parar()
        print("ERROR en el lazo, sigo:", e)
        if color_actual != ROJO:
            ib.pixel = ROJO
            color_actual = ROJO
        time.sleep(0.1)
