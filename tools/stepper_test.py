"""
Prueba individual de motores paso a paso (NEMA 17) del brazo robótico.

Mueve un solo motor un ángulo dado, medido en el EJE DEL MOTOR (no en la
articulación). Sirve para verificar cableado, pines y sentido de giro antes
de ejecutar move_to_point.py.

Mapeo de pines (igual que en move_to_point.py):
  Motor base (c)       -> STEP 25, DIR 24
  Motor eslabón 1 (d)  -> STEP 17, DIR 27
  Motor eslabón 2 (i)  -> STEP 20, DIR 26
"""
import time
import lgpio

# Pines
STEP_d = 17
DIR_d  = 27

STEP_i = 20
DIR_i  = 26

STEP_c = 25
DIR_c  = 24

# Parámetros del motor
steps_per_rev = 200      # pasos por vuelta (ajusta si usas microstepping)
step_delay = 0.006       # segundos

# Relaciones de transmisión (referencia: para convertir a ángulo de articulación,
# multiplica el ángulo de la articulación por la relación correspondiente)
Gear_Ratio_1  = 1 + (108 / 19)   # Base
Gear_Ratio_23 = 47 / 19          # Articulaciones 2 y 3

# Inicializar GPIO
h = lgpio.gpiochip_open(0)

for pin in [STEP_d, DIR_d, STEP_i, DIR_i, STEP_c, DIR_c]:
    lgpio.gpio_claim_output(h, pin)

def move_stepper(step_pin, dir_pin, angle_deg, direction=1):
    # angle_deg: ángulo en el eje del motor, en grados
    steps = int((angle_deg / 360.0) * steps_per_rev)

    lgpio.gpio_write(h, dir_pin, direction)

    for _ in range(steps):
        lgpio.gpio_write(h, step_pin, 1)
        time.sleep(step_delay)
        lgpio.gpio_write(h, step_pin, 0)
        time.sleep(step_delay)


try:
    move_stepper(STEP_c, DIR_c, 30, 0)    # motor base → 30°
    # move_stepper(STEP_d, DIR_d, 10, 1)  # motor eslabón 1 → 10°
    # move_stepper(STEP_i, DIR_i, 10, 1)  # motor eslabón 2 → 10°
finally:
    # Liberar GPIO
    lgpio.gpiochip_close(h)
    print("Movimiento terminado.")
