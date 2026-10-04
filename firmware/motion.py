"""motion.py — traduce decisiones en movimiento.

El controller no sabe nada de motores: devuelve dos números, `v` (avanzar) y
`w` (girar). Este archivo es el único que toca el hardware. Esa separación es
lo que permite probar toda la estrategia en la laptop: el simulador llama a
las mismas funciones del controller y aplica (v, w) con su propia física.

    controller  →  (v, w)  →  motion  →  motor_1, motor_2


EL CONVENIO DE SIGNOS  [medido, no supuesto]
--------------------------------------------
    v > 0  →  avanzar
    w > 0  →  girar ANTIHORARIO (a la izquierda)

El `w` antihorario no es capricho: es el mismo sentido en que crece `theta` en
la telemetría. Si uno de los dos fuera al revés, el lazo de control corregiría
el error agrandándolo, y el robot giraría cada vez más rápido en vez de
alinearse.

OJO CON EL EJEMPLO DE CENFOTEC
------------------------------
Su `codigos/test_motores.py` avanza así:

    ib.motor_1.throttle = speed
    ib.motor_2.throttle = speed      # los dos con el MISMO signo

En NUESTROS robots eso NO avanza: GIRA. Los motores están montados espejados,
así que hace falta signo opuesto. Lo comprobamos el 3 de octubre con una
prueba de cuatro combinaciones y un video: con signos iguales el robot giraba
sobre su eje.

No "arreglar" esto mirando el ejemplo de ellos. El ejemplo está bien para su
armado y mal para el nuestro.
"""

import config

# Por debajo de esto el pedido se considera cero. Sin este umbral, un error
# residual de centésimas mantendría los motores zumbando para siempre.
UMBRAL_PARAR = 0.02


def _limitar(x):
    return max(-1.0, min(1.0, x))


def _compensar_zona_muerta(x, zona_muerta):
    """Remapea el rango útil para saltarse la zona muerta.

    EL PROBLEMA
    -----------
    Por debajo de cierto throttle el motor zumba y el robot no se mueve. En
    nuestros robots, medido en el piso, eso pasa por debajo de 0.10.

    El lazo de control es proporcional: cuanto menor el error, menor la
    corrección que pide. Así que al acercarse al objetivo pide 0.09, 0.07,
    0.05... y el robot DEJA DE MOVERSE sin haber llegado. El error ya no baja,
    así que el lazo sigue pidiendo lo mismo, y el robot se queda ahí zumbando
    hasta que salta el timeout. Es la forma más tonta de perder una ronda.

    LA SOLUCIÓN
    -----------
    Estirar el rango: lo que el controller pide como "0.01 de potencia" se
    convierte en el mínimo que de verdad mueve, y lo que pide como "1.0" sigue
    siendo 1.0. Todo el rango intermedio se reparte entre esos dos.

        pedido:     0 ┤0.02├──────────────────────────────┤1.0
                       │                                   │
        enviado:    0 ┤0.10├──────────────────────────────┤1.0
                          ↑
                   zona muerta medida

    Así ningún valor que mande el controller cae en la zona donde no pasa nada.
    """
    if abs(x) < UMBRAL_PARAR:
        return 0.0
    signo = 1.0 if x > 0 else -1.0
    magnitud = zona_muerta + abs(x) * (1.0 - zona_muerta)
    return signo * magnitud


class Motores:

    def __init__(self, ib):
        self.ib = ib
        # Se leen de config porque son MEDIDOS y pueden diferir entre los dos
        # robots. El resto del código es idéntico en los dos.
        self.zona_muerta = getattr(config, "ZONA_MUERTA", 0.10)
        self.desvio = getattr(config, "COMPENSACION_DESVIO", 0.0)

    def parar(self):
        self.ib.motor_1.throttle = 0.0
        self.ib.motor_2.throttle = 0.0

    def aplicar(self, v, w):
        """v = avance (+ adelante), w = giro (+ antihorario, izquierda).

        MODELO DIFERENCIAL
        ------------------
        Un robot de dos ruedas gira porque una va más rápido que la otra. La
        cuenta es directa:

            rueda izquierda = v - w
            rueda derecha   = v + w

        Con w = 0 las dos van igual y el robot avanza recto. Con v = 0 y w > 0
        la izquierda va hacia atrás y la derecha hacia adelante: gira sobre su
        propio eje hacia la izquierda, sin desplazarse.

        El signo de motor_1 va invertido porque ese motor está montado al revés
        (ver el encabezado del archivo).
        """
        # Corrección de desvío: si el robot se va para un lado al avanzar
        # recto, esto lo endereza. Solo se aplica cuando de verdad está
        # avanzando: corregir un desvío mientras gira no tiene sentido.
        if abs(v) > UMBRAL_PARAR:
            w = w + self.desvio

        izq = _limitar(v - w)
        der = _limitar(v + w)

        izq = _compensar_zona_muerta(izq, self.zona_muerta)
        der = _compensar_zona_muerta(der, self.zona_muerta)

        self.ib.motor_1.throttle = _limitar(-izq)   # montado invertido
        self.ib.motor_2.throttle = _limitar(+der)