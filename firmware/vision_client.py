"""vision_client.py — recibe la telemetría en el robot.

Corre en el ESP32 con CircuitPython. Es lo único que toca la red.

QUÉ RESUELVE
------------
TCP no entrega mensajes: entrega un chorro de bytes. Un recv() puede traer
medio mensaje, tres mensajes y medio, o nada. Por eso hay que ACUMULAR en un
buffer y cortar por el salto de línea, que es lo que separa un mensaje del
siguiente en NDJSON.

Escribirlo sin buffer parece funcionar en la mesa de trabajo (los mensajes
chicos suelen llegar enteros) y falla en la cancha, cuando el tráfico
aprieta. Es el error clásico de este protocolo.

POLÍTICA: ÚLTIMO VALOR GANA
---------------------------
Si en un ciclo llegaron cinco mensajes, los cuatro primeros ya no sirven:
describen dónde estaba todo hace 200 ms. El robot necesita el más nuevo.

Por eso `ultimo()` vacía el socket y devuelve solo el último. No se encolan
mensajes: en tiempo real, la cola es latencia acumulada.
"""

import json
import time

import socketpool
import wifi


class ClienteVision:

    def __init__(self, ssid, password, host, puerto=2026):
        self.ssid = ssid
        self.password = password
        self.host = host
        self.puerto = puerto

        self.pool = None
        self.sock = None
        self.buffer = b""
        self.ultimo_ms = 0          # cuándo llegó el último mensaje
        self.recibidos = 0

    # ----------------------------------------------------------------------
    # Conexión
    # ----------------------------------------------------------------------

    def conectar_wifi(self):
        """Se conecta al router. Reintenta hasta lograrlo.

        No tiene sentido rendirse: sin Wi-Fi el robot no puede hacer nada,
        así que insiste.
        """
        while not wifi.radio.connected:
            print("wifi: conectando a", self.ssid)
            try:
                wifi.radio.connect(self.ssid, self.password)
            except Exception as e:
                print("wifi: falló (", e, ") reintento en 2 s")
                time.sleep(2)
        print("wifi: ok, ip", wifi.radio.ipv4_address)
        self.pool = socketpool.SocketPool(wifi.radio)

    def conectar(self):
        """Abre el socket contra el servidor de visión.

        El socket NO bloquea: si no hay datos, recv_into levanta una
        excepción en vez de quedarse esperando. Eso es clave, porque el lazo
        de control tiene que seguir corriendo aunque la red esté callada.
        """
        if self.pool is None:
            self.conectar_wifi()

        while True:
            try:
                print("vision: conectando a {}:{}".format(self.host, self.puerto))
                s = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)
                s.settimeout(0.01)          # casi sin bloqueo
                s.connect((self.host, self.puerto))
                self.sock = s
                self.buffer = b""
                print("vision: conectado")
                return
            except Exception as e:
                print("vision: falló (", e, ") reintento en 1 s")
                time.sleep(1)

    def reconectar(self):
        """La conexión se cayó. Cerrar y volver a abrir.

        Pasa: el router se satura, la PC de visión se reinicia, el Wi-Fi
        parpadea. El robot no puede quedarse muerto por eso.
        """
        print("vision: reconectando")
        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass
        self.sock = None
        self.conectar()

    # ----------------------------------------------------------------------
    # Recepción
    # ----------------------------------------------------------------------

    def ultimo(self):
        """Devuelve el mensaje MÁS NUEVO que llegó, o None si no llegó nada.

        Vacía el socket en cada llamada: si hay cinco mensajes esperando, los
        lee todos y devuelve el quinto. Los cuatro anteriores se descartan
        porque ya están viejos.
        """
        if self.sock is None:
            return None

        # 1. Traer todo lo que haya, sin bloquear.
        while True:
            try:
                chunk = bytearray(1024)
                n = self.sock.recv_into(chunk)
                if n == 0:
                    # El servidor cerró del otro lado.
                    self.reconectar()
                    return None
                self.buffer += bytes(chunk[:n])
            except OSError:
                # No hay más datos ahora mismo. Normal, no es error.
                break
            except Exception:
                self.reconectar()
                return None

            # Un tope por si el servidor manda más rápido de lo que leemos.
            if len(self.buffer) > 16384:
                break

        # 2. Cortar por saltos de línea y quedarse con la ÚLTIMA completa.
        msg = None
        while b"\n" in self.buffer:
            linea, self.buffer = self.buffer.split(b"\n", 1)
            linea = linea.strip()
            if not linea:
                continue
            try:
                msg = json.loads(linea)
                self.recibidos += 1
            except Exception:
                # Línea cortada o basura: se descarta y se sigue. Un mensaje
                # perdido no importa, llegan veinte por segundo.
                pass

        if msg is not None:
            self.ultimo_ms = time.monotonic_ns() // 1000000

        return msg

    # ----------------------------------------------------------------------
    # Salud de la conexión
    # ----------------------------------------------------------------------

    def edad_ms(self):
        """Cuánto hace que no llega un mensaje.

        Es el watchdog: si esto crece, el robot está navegando a ciegas y
        hay que frenar los motores.
        """
        if self.ultimo_ms == 0:
            return 999999
        return (time.monotonic_ns() // 1000000) - self.ultimo_ms