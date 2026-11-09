import sympy as sp
import numpy as np
from IK import solve_ik
import time
import lgpio

# =========================================================
# IK INPUTS
# =========================================================
Xref_val = 100
Yref_val = -10
Zref_val = 50

q1_rad, q2_rad, q3_rad = solve_ik(Xref_val, Yref_val, Zref_val)

q1_deg = np.rad2deg(q1_rad)
q2_deg = np.rad2deg(q2_rad)
q3_deg = np.rad2deg(q3_rad)

print("Imported from IK:")
print(f"q1 = {q1_deg:.3f} deg")
print(f"q2 = {q2_deg:.3f} deg")
print(f"q3 = {q3_deg:.3f} deg")

# =========================================================
# GEOMETRY SOLVE FOR theta1
# =========================================================
theta2_deg = q2_deg
phi_target_deg = 180 + q3_deg

theta1_min, theta1_max = -20, 180

L1, L2, L3, L4 = 55, 130, 145, 55

theta1_deg = sp.symbols('theta1_deg', real=True)

t1 = theta1_deg * sp.pi / 180
t2 = theta2_deg * sp.pi / 180

v1 = sp.Matrix([L1*sp.sin(t1), -L1*sp.cos(t1)])
v2 = sp.Matrix([L2*sp.sin(t2),  L2*sp.cos(t2)])
v3 = v2 - v1

mag_v3 = sp.sqrt(v3.dot(v3))

b1 = (mag_v3**2 + L1**2 - L2**2) / (2*mag_v3*L1)
z2 = (L4**2 + mag_v3**2 - L3**2) / (2*L4*mag_v3)

phi_expr = (
    180 - theta2_deg - theta1_deg
    + sp.acos(b1)*180/sp.pi
    - sp.acos(z2)*180/sp.pi
)

theta1_solution = None
for guess in np.linspace(theta1_min, theta1_max, 9):
    try:
        sol = float(sp.nsolve(phi_expr - phi_target_deg, theta1_deg, guess))
        if theta1_min <= sol <= theta1_max:
            theta1_solution = sol
            break
    except:
        pass

if theta1_solution is None:
    raise RuntimeError("No valid theta1 found")

print(f"Solved theta1 = {theta1_solution:.3f} deg")

# =========================================================
# STEPPER + GPIO SETUP
# =========================================================
Gear_Ratio_1 = 1 + (108/19)
Gear_Ratio_23 = 47/19
Pasos_por_grado = 200/360

STEP_1, DIR_1 = 25, 24
STEP_2, DIR_2 = 17, 27
STEP_3, DIR_3 = 20, 26

tiempo_pulso = 0.05

# Motor polarity (FIXES return bug)
MOTOR1_SIGN = +1
MOTOR2_SIGN = +1
MOTOR3_SIGN = -1   # inverted wiring

chip = lgpio.gpiochip_open(0)
for pin in [STEP_1, DIR_1, STEP_2, DIR_2, STEP_3, DIR_3]:
    lgpio.gpio_claim_output(chip, pin, 0)

# =========================================================
# HELPERS
# =========================================================
def angle_to_steps(angle_deg, steps_per_deg):
    return int(round(abs(angle_deg) * steps_per_deg))

def joint_to_stepper_deg_joint1(q):
    return q * Gear_Ratio_1

def joint_to_stepper_deg_joint23(q):
    return q * Gear_Ratio_23

def step_motors_sync(chip, motors, pulse_s):
    for _, dir_pin, _, direction in motors:
        lgpio.gpio_write(chip, dir_pin, int(direction))

    max_steps = max(m[2] for m in motors)

    for i in range(max_steps):
        for step_pin, _, steps, _ in motors:
            if i < steps:
                lgpio.gpio_write(chip, step_pin, 1)
        time.sleep(pulse_s)
        for step_pin, _, steps, _ in motors:
            if i < steps:
                lgpio.gpio_write(chip, step_pin, 0)
        time.sleep(pulse_s)

# =========================================================
# MOTION CONTROL
# =========================================================
current_q1_deg = 0.0
current_theta1_deg = 0.0
current_q2_deg = 90.0

start_q1_deg = current_q1_deg
start_theta1_deg = current_theta1_deg
start_q2_deg = current_q2_deg

def move_robot_to_angles(q1_t, th1_t, q2_t):
    global current_q1_deg, current_theta1_deg, current_q2_deg

    dq1 = q1_t - current_q1_deg
    dth1 = th1_t - current_theta1_deg
    dq2 = q2_t - current_q2_deg

    dstep1 = joint_to_stepper_deg_joint1(dq1)
    dstep2 = joint_to_stepper_deg_joint23(dth1)
    dstep3 = joint_to_stepper_deg_joint23(dq2)

    steps1 = angle_to_steps(dstep1, Pasos_por_grado)
    steps2 = angle_to_steps(dstep2, Pasos_por_grado)
    steps3 = angle_to_steps(dstep3, Pasos_por_grado)

    dir1 = 1 if MOTOR1_SIGN * dstep1 >= 0 else 0
    dir2 = 1 if MOTOR2_SIGN * dstep2 >= 0 else 0
    dir3 = 1 if MOTOR3_SIGN * dstep3 >= 0 else 0

    print("\n--- Motion plan ---")
    print(f"Target joints [deg]: q1={q1_t:.3f}, theta1={th1_t:.3f}, q2={q2_t:.3f}")
    print(f"Delta joints  [deg]: dq1={dq1:.3f}, dth1={dth1:.3f}, dq2={dq2:.3f}")
    print(f"Delta stepper [deg]: m1={dstep1:.3f}, m2={dstep2:.3f}, m3={dstep3:.3f}")
    print(f"Steps: m1={steps1}, m2={steps2}, m3={steps3}")

    motors = [
        (STEP_1, DIR_1, steps1, dir1),
        (STEP_2, DIR_2, steps2, dir2),
        (STEP_3, DIR_3, steps3, dir3),
    ]

    step_motors_sync(chip, motors, tiempo_pulso)

    current_q1_deg = q1_t
    current_theta1_deg = th1_t
    current_q2_deg = q2_t

# =========================================================
# EXECUTION
# =========================================================
try:
    move_robot_to_angles(q1_deg, theta1_solution, q2_deg)
    time.sleep(2)
    move_robot_to_angles(start_q1_deg, start_theta1_deg, start_q2_deg)

finally:
    lgpio.gpiochip_close(chip)