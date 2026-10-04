# config_ejemplo.py — plantilla. Copiar como config.py en CADA robot y editar.
#
# Es lo UNICO distinto entre los dos robots. El resto del codigo es identico.
#
# Si los dos quedan con MI_ID = 10, los dos calculan el reparto como si
# fueran el mismo robot, toman la misma lista y van por el mismo cubo.
# No da ningun error: funciona mal, en silencio.

# --- identidad -----------------------------------------------------------
# Tiene que coincidir con el marcador ArUco pegado arriba del robot.
MI_ID = 10                  # 10 en un robot, 11 en el otro
ID_COMPANERO = 11           # el del otro robot

# El rover con prioridad NUNCA cede el paso. Es asimetrico a proposito: si
# los dos cedieran, se quedarian trabados mirandose.
TIENE_PRIORIDAD = True      # True en el 10, False en el 11

# Los dos salen del mismo punto, y ese es el choque mas probable. El 11
# espera 3 segundos. Se cuenta desde READY.
RETARDO_SALIDA_MS = 0       # 0 en el 10, 3000 en el 11

# --- red -----------------------------------------------------------------
# El ESP32 solo habla 2.4 GHz. Una red de 5 GHz no conecta nunca, aunque la
# clave este bien.
WIFI_SSID = "PONER_AQUI"
WIFI_PASSWORD = "PONER_AQUI"

# La IP de la COMPUTADORA que corre el sistema de vision, no la de la camara
# (la camara es USB, no tiene IP). Se saca con ipconfig en esa maquina.
# Nunca 127.0.0.1: el robot es otro aparato en la red.
#
# OJO: ESTE NUMERO CAMBIA EN EL TORNEO. Es la IP de la laptop en la red de
# CENFOTEC, que no es la de la casa. Hay que preguntarla y ponerla en LOS DOS
# robots antes de la primera ronda.
#
# Este archivo tenia 192.168.1.100 —un valor de ejemplo que nunca existio—
# mientras config_robot11.py tenia la IP de verdad. Un robot conectaba y el
# otro no, y el sintoma era identico a un problema de firewall.
VISION_HOST = "192.168.100.3"     # [CAMBIAR EN EL TORNEO]
VISION_PUERTO = 2026

# --- motores  [LLENAR CON LO QUE SALGA DE p5] ----------------------------
# Estos numeros son de ESTE robot, no del modelo. Dos motores iguales nunca
# giran igual.
#
# ZONA_MUERTA: por debajo de este throttle los motores zumban y no mueven.
# Es el numero mas importante de p5: sin el, el robot se queda clavado a dos
# celdas del objetivo cuando el lazo baja la potencia al acercarse.
ZONA_MUERTA = 0.10         # [MEDIR con p5, medicion 1]

# Si un motor es mas fuerte que el otro, el robot curva avanzando derecho.
# Positivo = el robot se va a la izquierda, hay que frenar el derecho.
COMPENSACION_DESVIO = 0.0   # [MEDIR con p5, medicion 2]