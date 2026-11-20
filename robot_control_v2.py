import time
import cyberpi
import mbot2
import gamepad

# ---------------- SETTINGS (tune these) ----------------
DRIVE_SPEED   = 80    # max wheel power (0-100)
TURN_SPEED    = 35    # max turning power
DEADZONE      = 12    # ignore small joystick drift
LIFT_POWER    = 60    # M1 lift motor power
SHOOT_POWER   = 100   # M2 shooter wheel power

S1_MIN, S1_MAX = 0, 180     # Z-axis servo limits
S2_MIN, S2_MAX = 20, 160    # gripper servo limits (adjust to your gripper!)
S1_STEP = 3                 # degrees per loop while button held
S2_STEP = 4

# start positions
s1_angle = 90
s2_angle = 90
# --------------------------------------------------------

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def deadzone(v):
    return 0 if abs(v) < DEADZONE else v

mbot2.servo_set(s1_angle, "S1")
mbot2.servo_set(s2_angle, "S2")
cyberpi.console.println("Gamepad robot ready")

while True:
    # ---------- STOP ALL ----------
    if gamepad.is_key_pressed('Select'):
        mbot2.drive_power(0, 0)
        mbot2.motor_set(0, "M1")
        mbot2.motor_set(0, "M2")
        time.sleep(0.1)
        continue

    # ---------- DRIVING (EM1 / EM2) ----------
    fwd  = deadzone(gamepad.get_joystick('Ly'))   # up = positive
    turn = deadzone(gamepad.get_joystick('Lx'))   # right = positive

    fwd  = fwd  * DRIVE_SPEED / 100
    turn = turn * TURN_SPEED  / 100

    left  = clamp(fwd + turn, -100, 100)
    right = clamp(fwd - turn, -100, 100)

    # EM1 = left wheel, EM2 = right wheel (right is mirrored, so negate)
    mbot2.drive_power(left, -right)

    # ---------- LIFT (M1) ----------
    if gamepad.is_key_pressed('Up'):
        mbot2.motor_set(LIFT_POWER, "M1")
    elif gamepad.is_key_pressed('Down'):
        mbot2.motor_set(-LIFT_POWER, "M1")
    else:
        mbot2.motor_set(0, "M1")

    # ---------- SHOOTER WHEEL (M2) ----------
    if gamepad.is_key_pressed('R1'):
        mbot2.motor_set(SHOOT_POWER, "M2")
    elif gamepad.is_key_pressed('R2'):
        mbot2.motor_set(-SHOOT_POWER, "M2")
    else:
        mbot2.motor_set(0, "M2")

    # ---------- Z-AXIS SERVO (S1) ----------
    if gamepad.is_key_pressed('N1'):
        s1_angle = clamp(s1_angle + S1_STEP, S1_MIN, S1_MAX)
        mbot2.servo_set(s1_angle, "S1")
    elif gamepad.is_key_pressed('N4'):
        s1_angle = clamp(s1_angle - S1_STEP, S1_MIN, S1_MAX)
        mbot2.servo_set(s1_angle, "S1")

    # ---------- GRIPPER SERVO (S2) ----------
    if gamepad.is_key_pressed('N2'):      # close / grip
        s2_angle = clamp(s2_angle - S2_STEP, S2_MIN, S2_MAX)
        mbot2.servo_set(s2_angle, "S2")
    elif gamepad.is_key_pressed('N3'):    # open / release
        s2_angle = clamp(s2_angle + S2_STEP, S2_MIN, S2_MAX)
        mbot2.servo_set(s2_angle, "S2")

    time.sleep(0.02)
