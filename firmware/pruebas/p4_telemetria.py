# PRUEBA 4 — el robot entiende la telemetria
#
# Necesita: wifi y el sistema de vision corriendo.
#
# RESPONDE: el robot parsea el JSON, se encuentra a si mismo por id, y a
# que velocidad corre el lazo?
#
# EL NUMERO CLAVE SON LOS Hz. Si el lazo corre a 15 Hz o mas, CircuitPython
# alcanza. Si baja de 15, hay que evaluar migrar a C++, y eso cambia el plan
# de los dias que quedan.
#
# Correrla en LOS DOS robots a la vez: hay que ver si el servidor aguanta
# dos clientes sin que bajen los Hz.
#
# Contrato v3: cada cubo trae `in_depot`, el veredicto del arbitro.

import json
import time

import socketpool
import wifi
from ideaboard import IdeaBoard

import config

ib = IdeaBoard()
ib.pixel = (64, 0, 0)


def ahora_ms():
    return time.monotonic_ns() // 1000000


print()
print("=" * 40)
print("PRUEBA 4: telemetria")
print("=" * 40)
print()

while not wifi.radio.connected:
    print("wifi: conectando...")
    try:
        wifi.radio.connect(config.WIFI_SSID, config.WIFI_PASSWORD)
    except Exception as e:
        print("  fallo:", e)
        time.sleep(2)
print("wifi ok, ip", wifi.radio.ipv4_address)

pool = socketpool.SocketPool(wifi.radio)
sock = pool.socket(pool.AF_INET, pool.SOCK_STREAM)
sock.settimeout(0.01)          # casi sin bloqueo: el lazo no debe frenarse
sock.connect((config.VISION_HOST, config.VISION_PUERTO))
print("conectado a la vision")
ib.pixel = (0, 64, 0)
print()

buffer = b""
chunk = bytearray(1024)
recibidos = 0
invalidos = 0
saltos = 0
seq_previo = None
vueltas = 0

t_inicio = ahora_ms()
t_log = t_inicio

while True:
    vueltas += 1

    # 1. Traer TODO lo que haya, sin bloquear.
    while True:
        try:
            n = sock.recv_into(chunk)
            if n == 0:
                break
            buffer += bytes(chunk[:n])
        except OSError:
            break                      # no hay mas datos ahora: normal
        except Exception as e:
            print("error de red:", e)
            break
        if len(buffer) > 16384:
            break

    # 2. Cortar por salto de linea y quedarse con el ULTIMO completo.
    #    Los anteriores ya son viejos: en tiempo real la cola es latencia.
    msg = None
    while b"\n" in buffer:
        linea, buffer = buffer.split(b"\n", 1)
        linea = linea.strip()
        if not linea:
            continue
        try:
            msg = json.loads(linea)
            recibidos += 1
        except Exception:
            invalidos += 1

    if msg is not None:
        seq = msg.get("seq")
        if seq_previo is not None and seq != seq_previo + 1:
            saltos += 1       # normal: politica de ultimo-valor-gana
        seq_previo = seq

    # 3. Reporte cada segundo.
    t = ahora_ms()
    if t - t_log > 1000 and msg is not None:
        hz_lazo = vueltas * 1000.0 / max(1, t - t_inicio)
        hz_msg = recibidos * 1000.0 / max(1, t - t_inicio)

        yo = None
        for r in msg.get("rovers", []):
            if r["id"] == config.MI_ID:
                yo = r
                break
        pos = "({:.2f}, {:.2f}) {:.1f}deg age={}".format(
            yo["col"], yo["row"], yo["theta"], yo["age_ms"]) if yo else "NO ME VE"

        print("v{} | lazo {:.1f}Hz | msgs {:.1f}Hz | fase {} | saltos {}".format(
            msg.get("v"), hz_lazo, hz_msg, msg.get("phase"), saltos))
        print("   yo:", pos)
        for c in msg.get("cubes", []):
            print("   cubo {:5s} ({:.2f}, {:.2f}) age={} in_depot={}".format(
                c["color"], c["col"], c["row"], c["age_ms"],
                c.get("in_depot", "?")))
        print()
        t_log = t

    time.sleep(0.005)