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
telemetría. Durante IDLE el robot está encendido y conectado, quieto; en
cuanto ve READY, sale solo.

El LED dice en qué está, y eso se puede leer de un vistazo en la cancha:

    rojo      sin telemetría fresca (watchdog)
    amarillo  conectado, esperando READY
    verde     trabajando

EL WATCHDOG
-----------
Si deja de llegar telemetría, los motores van a CERO. Un robot ciego que
sigue avanzando se sale de la cancha o embiste al compañero; uno que frena
solo pierde unos segundos y se recupera cuando vuelve la señal.
"""

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
AMARILLO = (64, 64, 0)
VERDE = (0, 64, 0)


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
print("CONECTADO. Esperando READY por telemetría. No hay que apretar nada.")


# --------------------------------------------------------------------------
# Lazo principal
# --------------------------------------------------------------------------

t_ultimo_log = 0
vueltas = 0
t_inicio = ahora_ms()

while True:
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
        ib.pixel = ROJO
        time.sleep(0.02)
        continue

    # 3. DECIDIR. El controlador mira la fase y decide si le toca moverse.
    try:
        v, w = ctrl.paso(t)
    except Exception as e:
        # Un error en la lógica no puede dejar los motores encendidos.
        print("ERROR en controller:", e)
        motores.parar()
        time.sleep(0.1)
        continue

    # 4. MOVER.
    motores.aplicar(v, w)

    ib.pixel = VERDE if mundo.activa() else AMARILLO

    # 5. LOG, una vez por segundo.
    if t - t_ultimo_log > PERIODO_LOG_MS:
        hz = vueltas * 1000.0 / max(1, t - t_inicio)
        yo = mundo.yo()
        pos = "({:.1f},{:.1f})".format(yo["col"], yo["row"]) if yo else "??"
        print("{:5.1f}Hz | {} | {} | {} | obj={} | edad={}ms".format(
            hz, mundo.fase(), ctrl.estado, pos, ctrl.objetivo,
            cliente.edad_ms()))
        t_ultimo_log = t

    # 6. Respirar. El lazo no tiene que ir más rápido que la telemetría:
    #    correr a 200 Hz con datos que llegan a 20 Hz es gastar CPU para
    #    recalcular lo mismo veinte veces.
    time.sleep(0.01)