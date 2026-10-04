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

# Cada cuánto se reintenta la reconexión. No más seguido: cada intento
# fallido cuesta tiempo del lazo de control.
REINTENTO_MS = 1000

# Timeout para el HANDSHAKE TCP. Tiene que ser generoso: abrir una conexión
# por wifi lleva entre 5 y 50 ms de rutina, y con la red cargada más.
TIMEOUT_CONECTAR_S = 5.0

# Timeout para RECIBIR, una vez conectado. Acá sí tiene que ser casi cero: el
# lazo de control no puede quedarse esperando datos que todavía no llegaron.
TIMEOUT_RECIBIR_S = 0.01

# Tras este silencio se da la conexión por muerta y se reconecta, aunque el
# socket parezca sano. Es el seguro de vida de todo esto: ver más abajo, en
# ultimo().
SIN_DATOS_MS = 3000


def _cerrar(sock):
    """Cierra un socket sin quejarse si ya estaba cerrado o es None.

    Se usa en TODOS los caminos de error. Un socket que no se cierra queda
    ocupado para siempre y el ESP32 tiene muy pocos.
    """
    if sock is None:
        return
    try:
        sock.close()
    except Exception:
        pass


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
        self.ultimo_intento_ms = 0  # cuándo se intentó reconectar por última vez

        # Último momento en que SUPIMOS que la conexión estaba viva: o llegó
        # un mensaje, o acabamos de conectar. Es distinto de `ultimo_ms`, que
        # solo mira mensajes y es lo que usa el watchdog de los motores.
        self.senal_ms = 0

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
            s = None
            try:
                print("vision: conectando a {}:{}".format(self.host, self.puerto))
                s = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)

                # El orden importa. Si se pone el timeout corto ANTES del
                # connect, el handshake TCP hereda esos 10 ms y falla con
                # ETIMEDOUT aunque el servidor esté perfecto del otro lado.
                s.settimeout(TIMEOUT_CONECTAR_S)
                s.connect((self.host, self.puerto))
                s.settimeout(TIMEOUT_RECIBIR_S)   # recién ahora, el corto

                self.sock = s
                self.buffer = b""
                self.senal_ms = time.monotonic_ns() // 1000000
                print("vision: conectado")
                return
            except Exception as e:
                # CERRAR EL SOCKET FALLIDO, SIEMPRE.
                #
                # El ESP32 tiene una cantidad fija de sockets (unos 4 a 8).
                # Un socket que se crea y no se cierra queda ocupado para
                # siempre. Sin este cierre, cada intento fallido se come uno,
                # y a los pocos segundos el robot muere con "Out of sockets":
                # ya no puede abrir ninguno, aunque el servidor esté bien.
                # Ni reiniciar el wifi lo salva, solo reiniciar la placa.
                _cerrar(s)
                print("vision: falló (", e, ") reintento en 1 s")
                time.sleep(1)

    def reconectar(self):
        """La conexión se cayó. Intenta UNA vez y vuelve enseguida.

        POR QUÉ NO SE QUEDA REINTENTANDO ACÁ ADENTRO
        --------------------------------------------
        Antes esta función llamaba a `conectar()`, que reintenta en un bucle
        infinito. Eso dejaba el lazo de control COLGADO acá: `motores.aplicar()`
        no se volvía a llamar nunca, así que los motores se quedaban con el
        último valor que habían recibido y el robot seguía andando a ciegas
        hasta chocar contra algo. Un robot sin telemetría tiene que FRENAR, no
        seguir derecho.

        Volviendo enseguida, el lazo sigue girando: el watchdog ve que la edad
        del dato crece, para los motores, y en el ciclo siguiente se vuelve a
        intentar la reconexión. El robot queda quieto y vivo hasta que la
        señal vuelva.
        """
        _cerrar(self.sock)
        self.sock = None
        self.buffer = b""

        if self.pool is None:
            return

        ahora = time.monotonic_ns() // 1000000
        if ahora - self.ultimo_intento_ms < REINTENTO_MS:
            return                      # todavía no toca reintentar
        self.ultimo_intento_ms = ahora

        s = None
        try:
            s = self.pool.socket(self.pool.AF_INET, self.pool.SOCK_STREAM)
            s.settimeout(TIMEOUT_CONECTAR_S)
            s.connect((self.host, self.puerto))
            s.settimeout(TIMEOUT_RECIBIR_S)
            self.sock = s
            self.senal_ms = ahora
            print("vision: reconectado")
        except Exception as e:
            _cerrar(s)          # ver la nota en conectar(): si no, Out of sockets
            print("vision: reconexión falló (", e, ")")

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
            # Sin socket: se intenta recuperarlo. reconectar() respeta su
            # propio intervalo, así que llamarla en cada ciclo no satura nada.
            self.reconectar()
            if self.sock is None:
                return None

        # CONEXIÓN MUERTA POR SILENCIO
        # ----------------------------
        # No alcanza con esperar a que recv_into devuelva 0. Con el timeout de
        # 10 ms, CircuitPython levanta OSError tanto cuando el servidor cerró
        # como cuando simplemente todavía no hay datos — y son indistinguibles
        # desde acá. Si solo confiáramos en eso, al caerse el servidor el robot
        # se quedaría con un socket inservible, repitiendo "sin telemetría"
        # para siempre, sin intentar reconectar nunca.
        #
        # El silencio sí es inequívoco: la visión publica a 20 Hz, o sea un
        # mensaje cada 50 ms. Tres segundos sin nada son sesenta mensajes
        # perdidos. Eso no es una pausa, es una conexión muerta.
        ahora = time.monotonic_ns() // 1000000
        if self.senal_ms and (ahora - self.senal_ms) > SIN_DATOS_MS:
            print("vision: silencio de", ahora - self.senal_ms, "ms, reconectando")
            self.reconectar()
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
            self.senal_ms = self.ultimo_ms

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