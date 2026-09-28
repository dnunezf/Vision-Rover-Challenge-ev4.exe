"""
grabar.py => guarda la telemetría en un archivo (corre en la PC, no en el robot)

Existe porque el sistema de vision no guarda la secuencia de mensajes. Sin
esto, cada prueba necesita la cancha montada y nunca sale igual dos veces.
Con esto, el que tiene la camara graba una sesion, manda el archivo, y el
resto puede trabajar con datos reales desde su laptop.

USO (con el simulador corriendo en otra ventana):
    py tools\\grabar.py --segundos 30
    py tools\\grabar.py --host 192.168.1.47 --segundos 300

"""

import argparse
import json
import os
import socket
import time

def main():

    """
    Los parámetros van por línea de comando y no escritos en el código,
    porque la IP cambia según donde esté corriendo la visión: en nuestra propia máquina es 
    127.0.0.1, pero en la cancha es la IP de la PC del compañero (David)
    """

    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--puerto", type=int, default=2026)
    ap.add_argument("--segundos", type=int, default=0)   # 0 = hasta Ctrl-C
    args = ap.parse_args()

    # El nombre lleva fecha y hora (para mayor control)

    os.makedirs("grabaciones", exist_ok=True)
    nombre = os.path.join("grabaciones", "sesion_" + time.strftime("%Y%m%d_%H%M%S") + ".ndjson")

    print("Conectado a {}:{} ...".format(args.host, args.puerto))
    sock = socket.create_connection((args.host, args.puerto), timeout=5)

    sock.settimeout(1.0)
    print("conectado. grabando en", nombre)

    t0 = time.monotonic()
    buffer = b""
    guardados = 0

    try:
        with open(nombre, "w", encoding="utf-8") as f:
            while True:
                if args.segundos and (time.monotonic() - t0) > args.segundos:
                    break

                try:
                    trozo = sock.recv(4096)
                except socket.timeout:
                    continue          # no llego nada en este segundo, seguimos
                if not trozo:
                    print("la vision cerro la conexion")
                    break
                 # TCP entrega un monton de bytes seguidos, sin marcar donde
                # termina un mensaje y empieza el otro. Una sola lectura puede
                # traer media linea, o dos lineas y media. Por eso se crea un
                # buffer donde se van acumulando los bytes segun llegan.
                buffer += trozo

                # Lo unico que marca el final de un mensaje es el salto de
                # linea que pone la vision. Entonces se corta el buffer en
                # cada '\n', y lo que queda despues del ultimo salto se
                # guarda para la proxima vuelta, porque es el principio de un
                # mensaje que todavia no llego completo.
                while b"\n" in buffer:
                    linea, buffer = buffer.split(b"\n", 1)
                    if not linea.strip():
                        continue
                    try:
                        msg = json.loads(linea)
                    except Exception:
                        continue      # linea rota: se descarta y se sigue

                    # Se anota el momento en que llego cada mensaje, porque
                    # despues, al reproducir la grabacion, hay que respetar
                    # esa misma cadencia. No sirve el ts_ms que trae el
                    # mensaje: ese es el reloj de la PC de vision y puede
                    # estar en cualquier hora.
                    msg["_t_ms"] = int((time.monotonic() - t0) * 1000)

                    # Se guarda una linea por mensaje, igual que como llego.
                    # Asi el archivo tiene el mismo formato que la vision y
                    # se puede volver a emitir tal cual.
                    f.write(json.dumps(msg) + "\n")
                    guardados += 1

                    if guardados % 200 == 0:
                        print("  {} mensajes | fase {}".format(
                            guardados, msg.get("phase")))
    except KeyboardInterrupt:
        print("\ncortado a mano")
    finally:
        sock.close()

    # La cadencia final dice si la grabacion sirve: tiene que dar cerca de
    # 20 Hz. Si dio mucho menos, se perdieron mensajes y conviene repetirla.
    dur = time.monotonic() - t0
    print()
    print("archivo  :", nombre)
    print("mensajes :", guardados)
    print("duracion : {:.1f} s".format(dur))
    print("cadencia : {:.1f} Hz".format(guardados / dur if dur else 0))


if __name__ == "__main__":
    main()


    

    

