"""config.py del ROBOT 11.

Es el ÚNICO archivo distinto entre los dos robots. Todo el resto del código es
idéntico, byte por byte.

Un robot con el config del otro no da ningún error: arranca, se conecta, y
hace el trabajo del compañero. Falla en silencio, que es la peor forma.
"""

# --- identidad -----------------------------------------------------------
# Tiene que coincidir con el marcador ArUco pegado arriba del robot.
MI_ID = 11
ID_COMPANERO = 10

# El rover con prioridad NUNCA cede el paso. Es asimétrico a propósito: si los
# dos cedieran, se quedarían trabados mirándose, como dos personas en un
# pasillo. Con prioridad fija, uno siempre avanza.
TIENE_PRIORIDAD = False

# Los dos salen del mismo punto y ese es el choque más probable de la ronda.
# El 11 espera 3 segundos. Se cuenta desde READY. Este robot espera 3 s.
RETARDO_SALIDA_MS = 3000

# --- red -----------------------------------------------------------------
# El ESP32 solo habla 2.4 GHz. Una red de 5 GHz no aparece ni siquiera al
# escanear, por más que la clave esté bien.
WIFI_SSID = "PONER_AQUI"
WIFI_PASSWORD = "PONER_AQUI"

# IP de la COMPUTADORA que corre la visión (o el mock_publisher), no la de la
# cámara: la cámara es USB y no tiene IP. Nunca 127.0.0.1, porque el robot es
# otro aparato en la red.
#
# Medido el 3-oct con 'ip addr' en la laptop de David, interfaz wlp0s20f3.
# CAMBIA al conectarse a otra red: hay que verificarla antes de cada sesión,
# y el MIERCOLES 7 en el torneo va a ser otra.
VISION_HOST = "192.168.100.3"

# Puerto oficial del contrato. No se toca.
VISION_PUERTO = 2026

# --- calibración de ESTE robot  [medido con p5, 3-oct] -------------------
# Throttle mínimo al que el robot se mueve APOYADO EN EL PISO. Por debajo, el
# motor zumba y no pasa nada. En el aire daba 0.08, pero vale el del piso: en
# el aire las ruedas no cargan el peso.
#
# Es el número más importante de la calibración. El lazo de control es
# proporcional, así que al acercarse al objetivo pide cada vez menos potencia;
# sin esta compensación el robot se clava a media celda del destino y se queda
# zumbando hasta que salta el timeout.
ZONA_MUERTA = 0.10

# Corrección de desvío al avanzar recto. Positivo si se va a la izquierda.
# Medido: 0.5 cm de desvío en 30 cm. Eso es menos de un grado y está dentro
# del error de medir con cinta, así que no se compensa. Compensar un desvío
# que no existe lo CREA.
COMPENSACION_DESVIO = 0.0