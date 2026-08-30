"""GR6 motor/pump/ultrasonic firmware - Raspberry Pi Pico (MicroPython)

Replaces drive/firmware/arduino/GR6_motor.ino. Same wire protocol (see
drive/protocol.py) so the Pi side needs no changes beyond serial_port in
config.yaml - control loop, PID gains, deadband/stiction handling and
the telemetry round-robin are all ported line-for-line from the Arduino
version, with two known copy/paste bugs fixed along the way (see
drive-prd.md's "Ported from Arduino" section):
  - RM_setMotorSpeed() was deadbanding against LM_Db instead of RM_Db.
  - The right motor's filtered D-term was decaying with LM_Ka instead
    of RM_Ka.
Both were silent while LM/RM gains defaulted equal - fixed here since
this is a fresh port, not a requirement to reproduce.

New for this board (see gr6_pins.py):
  - PIN_12V_PWR_EN latched high immediately, before anything else runs -
    this is what keeps 12V_PWR alive after the wake button is released.
  - PIN_MOTOR_STBY held high - the TB6612 driver chips (replacing the
    L298 module) are in standby until this is driven high.
  - Ultrasonics moved from the Arduino's analog pins onto the
    USR_1..USR_5 breakout (USR_6 spare, unused).
  - PIN_PUMP2_EN defined and driven low (off) at startup for a future
    second pump/device - no command wired to it yet.
"""

import select
import sys
import time

from machine import PWM, Pin, time_pulse_us

import gr6_pins as pins

VERSION = "260828#1.GR6"

# ---- Power latch: first thing we do, full stop ----
pwr_en = Pin(pins.PWR_12V_EN, Pin.OUT)
pwr_en.value(1)

# ---- Motor driver standby (TB6612s) ----
motor_stby = Pin(pins.MOTOR_STBY, Pin.OUT)
motor_stby.value(1)

# ---- Spare second pump/device output - off, unused for now ----
pump2_en = Pin(pins.PUMP2_EN, Pin.OUT)
pump2_en.value(0)

# ---- Motor pins ----
LM_IN1 = Pin(pins.LM_IN1, Pin.OUT)
LM_IN2 = Pin(pins.LM_IN2, Pin.OUT)
LM_PWM = PWM(Pin(pins.LM_PWM))
LM_PWM.freq(20000)  # above audible range; TB6612 handles it fine

RM_IN1 = Pin(pins.RM_IN1, Pin.OUT)
RM_IN2 = Pin(pins.RM_IN2, Pin.OUT)
RM_PWM = PWM(Pin(pins.RM_PWM))
RM_PWM.freq(20000)

# ---- Pump 1 ----
pump1_en = Pin(pins.PUMP1_EN, Pin.OUT)
pump1_en.value(0)
wp_state = 0

# ---- Ultrasonic sensors: USR_1..USR_5, single-pin trigger+echo (SR04,
# 3.3V-compatible) - USR_6 is spare, unused. ----
ULTRASONIC_PINS = [pins.USR_1, pins.USR_2, pins.USR_3, pins.USR_4, pins.USR_5]

# ---- Encoders ----
LM_ENC_A = Pin(pins.LM_ENC_A, Pin.IN, Pin.PULL_UP)
LM_ENC_B = Pin(pins.LM_ENC_B, Pin.IN, Pin.PULL_UP)
RM_ENC_A = Pin(pins.RM_ENC_A, Pin.IN, Pin.PULL_UP)
RM_ENC_B = Pin(pins.RM_ENC_B, Pin.IN, Pin.PULL_UP)

LM_position = 0
RM_position = 0

# Direction decode: see NOTES.md "Adaptive (ratio-based) decode
# (2026-08-30)". Alternation order (A,B,A,B,...) by itself carries no
# direction information - it happens every cycle regardless of which
# way the wheel is turning, since these are two phase-offset unipolar
# sensors, not overlapping ~50%-duty channels. What's actually
# asymmetric is the GAP: ~45 degrees (short) vs ~135 degrees (long, the
# wrap-around) - only the short transition reveals which channel is
# really leading, so only it is counted; the long one is skipped to
# avoid double-counting/flip-flop. The short:long ratio (~1:3) holds at
# any speed, but the absolute durations scale directly with speed - a
# fixed microsecond cutoff had a hard cliff below ~56 rises/s (reported
# exactly zero despite real motion). Classifying each transition
# against the PREVIOUS transition's duration instead (short = under
# ENC_RATIO of it) removed that cliff entirely - bench-confirmed
# accurate at every speed down to each motor's own stall point, with
# no absolute number baked in.
ENC_RATIO = 0.5
ENC_STALE_GAP_US = 1_000_000  # idle this long - nothing to compare against, same as a fresh start

_lm_last_channel = None
_lm_last_time = 0
_lm_prev_dt = None
_rm_last_channel = None
_rm_last_time = 0
_rm_prev_dt = None


def _lm_encoder_edge(channel):
    global LM_position, _lm_last_channel, _lm_last_time, _lm_prev_dt
    now = time.ticks_us()
    if _lm_last_channel is None:
        _lm_last_channel = channel
        _lm_last_time = now
        return
    dt = time.ticks_diff(now, _lm_last_time)
    if dt > ENC_STALE_GAP_US:
        _lm_prev_dt = None
    elif channel != _lm_last_channel and _lm_prev_dt is not None:
        if dt < ENC_RATIO * _lm_prev_dt:
            if _lm_last_channel == "A" and channel == "B":
                LM_position -= 1
            else:
                LM_position += 1
    if channel != _lm_last_channel:
        _lm_prev_dt = dt
    _lm_last_channel = channel
    _lm_last_time = now


def _lm_enc_a_isr(pin):
    _lm_encoder_edge("A")


def _lm_enc_b_isr(pin):
    _lm_encoder_edge("B")


# Sign confirmed on real amundsen hardware 2026-08-29 (see NOTES.md's
# "Sign convention confirmed on real hardware") after the motor lead
# polarity was swapped on both motors - LM needs the mirrored
# convention and RM the plain one, opposite of the original guess.
def _rm_encoder_edge(channel):
    global RM_position, _rm_last_channel, _rm_last_time, _rm_prev_dt
    now = time.ticks_us()
    if _rm_last_channel is None:
        _rm_last_channel = channel
        _rm_last_time = now
        return
    dt = time.ticks_diff(now, _rm_last_time)
    if dt > ENC_STALE_GAP_US:
        _rm_prev_dt = None
    elif channel != _rm_last_channel and _rm_prev_dt is not None:
        if dt < ENC_RATIO * _rm_prev_dt:
            if _rm_last_channel == "A" and channel == "B":
                RM_position += 1
            else:
                RM_position -= 1
    if channel != _rm_last_channel:
        _rm_prev_dt = dt
    _rm_last_channel = channel
    _rm_last_time = now


def _rm_enc_a_isr(pin):
    _rm_encoder_edge("A")


def _rm_enc_b_isr(pin):
    _rm_encoder_edge("B")


LM_ENC_A.irq(trigger=Pin.IRQ_RISING, handler=_lm_enc_a_isr)
LM_ENC_B.irq(trigger=Pin.IRQ_RISING, handler=_lm_enc_b_isr)
RM_ENC_A.irq(trigger=Pin.IRQ_RISING, handler=_rm_enc_a_isr)
RM_ENC_B.irq(trigger=Pin.IRQ_RISING, handler=_rm_enc_b_isr)

# ---- Control loop constants ----
CTRL_LOOP_STEP_MS = 100
dt = CTRL_LOOP_STEP_MS / 1000.0
cf = 1.0 / dt

LM_Kp, RM_Kp = 1.0, 1.0
LM_Ki, RM_Ki = 3.0, 3.0
LM_Kd, RM_Kd = 0.0, 0.0
LM_Kf, RM_Kf = 0.0, 0.0
LM_Ka, RM_Ka = 0.9, 0.9
LM_Kb, RM_Kb = 0.6, 0.6
LM_Mi, RM_Mi = 150.0, 150.0
LM_Mj, RM_Mj = -50.0, -50.0
LM_Id, RM_Id = 0.97, 0.97
LM_Db, RM_Db = 5, 5
LM_Am, RM_Am = 100.0 * dt, 100.0 * dt

prevLM_pos, prevRM_pos = 0, 0
filtLM_vel, filtRM_vel = 0.0, 0.0
prev_errLM, prev_errRM = 0.0, 0.0
filtLM_dErr, filtRM_dErr = 0.0, 0.0
integralLM, integralRM = 0.0, 0.0
ctrlEnabled = 0

MotorMax = 250.0

LM_target_setvel, RM_target_setvel = 0.0, 0.0
LM_setvel, RM_setvel = 0.0, 0.0

CMD_TIMEOUT_MS = 2000
TimeSpeedLastCommand = 0
TimePumpLastCommand = 0

whichUpdate = 1
whichControlUpdate = 1


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def set_motor_speed(motor_speed, in1, in2, pwm, deadband):
    abs_speed = abs(motor_speed)
    if abs_speed <= deadband:
        abs_speed = 0
    elif abs_speed < 30:
        abs_speed += 30  # overcome stiction
    if abs_speed > MotorMax:
        abs_speed = MotorMax

    if motor_speed > 0:
        in1.value(1)
        in2.value(0)
    else:
        in1.value(0)
        in2.value(1)

    pwm.duty_u16(int(abs_speed * 65535 / 255))


def lm_set_motor_speed(motor_speed):
    set_motor_speed(motor_speed, LM_IN1, LM_IN2, LM_PWM, LM_Db)


def rm_set_motor_speed(motor_speed):
    set_motor_speed(motor_speed, RM_IN1, RM_IN2, RM_PWM, RM_Db)


def send_float(field, v1, v2):
    print("%s %d %d" % (field, int(v1 * 100.0), int(v2 * 100.0)))


def send_int(field, v1, v2):
    print("%s %d %d" % (field, v1, v2))


def send_long(field, v1, v2):
    print("%s %d %d" % (field, v1, v2))


def send_ultrasonic(sensor):
    pin_num = ULTRASONIC_PINS[sensor]
    trig_pin = Pin(pin_num, Pin.OUT)
    trig_pin.value(0)
    time.sleep_us(2)
    trig_pin.value(1)
    time.sleep_us(10)
    trig_pin.value(0)

    echo_pin = Pin(pin_num, Pin.IN)
    duration_us = time_pulse_us(echo_pin, 1, 13000)  # timeout ~2m, matches Arduino

    if duration_us < 0:
        d = -1
    else:
        d = (duration_us * 343) // 2000  # mm, two-way
        if d > 2000 or d == 0:
            d = -1

    print("U%d %d" % (sensor, d))


def user_command(cmd):
    """Parse one command line (see drive/protocol.py's encode_* for the
    wire format this must accept)."""
    global LM_target_setvel, RM_target_setvel, ctrlEnabled, TimeSpeedLastCommand
    global wp_state, TimePumpLastCommand
    global LM_Kp, RM_Kp, LM_Ki, RM_Ki, LM_Kd, RM_Kd, LM_Kf, RM_Kf, LM_Ka, RM_Ka
    global LM_Kb, RM_Kb, LM_Db, RM_Db, LM_Mi, RM_Mi, LM_Mj, RM_Mj, LM_Id, RM_Id
    global LM_Am, RM_Am

    now = time.ticks_ms()
    parts = cmd.split()
    if not parts:
        return
    tag = parts[0]

    try:
        if tag == "SV" and len(parts) == 3:
            l1, l2 = int(parts[1]), int(parts[2])
            LM_target_setvel = float(clamp(l1, -200, 200))
            RM_target_setvel = float(clamp(l2, -200, 200))
            if l1 == 0 and l2 == 0:
                ctrlEnabled = 0
                LM_target_setvel = 0.0
                RM_target_setvel = 0.0
                lm_set_motor_speed(0)
                rm_set_motor_speed(0)
            elif ctrlEnabled == 0:
                ctrlEnabled = 1
            TimeSpeedLastCommand = now

        elif tag == "WP" and len(parts) == 2:
            wp_state = 1 if int(parts[1]) else 0
            pump1_en.value(wp_state)
            TimePumpLastCommand = now

        elif tag == "Kp" and len(parts) == 3:
            LM_Kp, RM_Kp = int(parts[1]) * 0.01, int(parts[2]) * 0.01
        elif tag == "Ki" and len(parts) == 3:
            LM_Ki, RM_Ki = int(parts[1]) * 0.01, int(parts[2]) * 0.01
        elif tag == "Kd" and len(parts) == 3:
            LM_Kd, RM_Kd = int(parts[1]) * 0.01, int(parts[2]) * 0.01
        elif tag == "Kf" and len(parts) == 3:
            LM_Kf, RM_Kf = int(parts[1]) * 0.01, int(parts[2]) * 0.01
        elif tag == "Ka" and len(parts) == 3:
            LM_Ka, RM_Ka = int(parts[1]) * 0.01, int(parts[2]) * 0.01
        elif tag == "Kb" and len(parts) == 3:
            LM_Kb, RM_Kb = int(parts[1]) * 0.01, int(parts[2]) * 0.01
        elif tag == "Db" and len(parts) == 3:
            LM_Db, RM_Db = int(parts[1]), int(parts[2])
        elif tag == "Mi" and len(parts) == 3:
            LM_Mi, RM_Mi = float(parts[1]), float(parts[2])
        elif tag == "Mj" and len(parts) == 3:
            LM_Mj, RM_Mj = float(parts[1]), float(parts[2])
        elif tag == "Id" and len(parts) == 3:
            LM_Id, RM_Id = int(parts[1]) * 0.01, int(parts[2]) * 0.01
        elif tag == "Am" and len(parts) == 3:
            LM_Am, RM_Am = float(parts[1]) * dt, float(parts[2]) * dt
    except ValueError:
        pass  # malformed command - ignore, same as the Arduino's sscanf failing silently


def control_step():
    global LM_setvel, RM_setvel, prevLM_pos, prevRM_pos
    global filtLM_vel, filtRM_vel, errLM, errRM, integralLM, integralRM
    global filtLM_dErr, filtRM_dErr, prev_errLM, prev_errRM
    global outLM, outRM, ctrlEnabled

    dLM = clamp(LM_target_setvel - LM_setvel, -LM_Am, LM_Am)
    dRM = clamp(RM_target_setvel - RM_setvel, -RM_Am, RM_Am)
    LM_setvel += dLM
    RM_setvel += dRM

    rawLM_vel = (LM_position - prevLM_pos) * cf
    rawRM_vel = (RM_position - prevRM_pos) * cf
    prevLM_pos = LM_position
    prevRM_pos = RM_position

    filtLM_vel = LM_Kb * filtLM_vel + (1.0 - LM_Kb) * rawLM_vel
    filtRM_vel = RM_Kb * filtRM_vel + (1.0 - RM_Kb) * rawRM_vel

    errLM = LM_setvel - filtLM_vel
    errRM = RM_setvel - filtRM_vel

    integralLM *= LM_Id
    integralRM *= RM_Id
    integralLM = clamp(integralLM + errLM * dt, LM_Mj, LM_Mi)
    integralRM = clamp(integralRM + errRM * dt, RM_Mj, RM_Mi)

    dErrLM = (errLM - prev_errLM) * cf
    dErrRM = (errRM - prev_errRM) * cf
    filtLM_dErr = filtLM_dErr * LM_Ka + (1.0 - LM_Ka) * dErrLM
    filtRM_dErr = filtRM_dErr * RM_Ka + (1.0 - RM_Ka) * dErrRM
    prev_errLM = errLM
    prev_errRM = errRM

    outLMf = LM_Kf * LM_setvel + LM_Kp * errLM + LM_Ki * integralLM + LM_Kd * filtLM_dErr
    outRMf = RM_Kf * RM_setvel + RM_Kp * errRM + RM_Ki * integralRM + RM_Kd * filtRM_dErr
    outLM = int(outLMf)
    outRM = int(outRMf)

    if ctrlEnabled > 0:
        # ctrlEnabled ramps 1->4 at start to avoid glitches
        outLM = ctrlEnabled * outLM // 4
        outRM = ctrlEnabled * outRM // 4
        ctrlEnabled = 4 if ctrlEnabled >= 4 else ctrlEnabled + 1
        lm_set_motor_speed(outLM)
        rm_set_motor_speed(outRM)
    else:
        integralLM *= 0.3
        integralRM *= 0.3


outLM, outRM = 0, 0
errLM, errRM = 0.0, 0.0


def send_telemetry():
    global whichUpdate, whichControlUpdate

    if whichUpdate == 1:
        send_long("EN", LM_position, RM_position)
    elif whichUpdate == 2:
        send_float("SV", LM_setvel, RM_setvel)
    elif whichUpdate == 3:
        send_float("FV", filtLM_vel, filtRM_vel)
    elif whichUpdate == 4:
        send_float("ER", errLM, errRM)
    elif whichUpdate == 5:
        send_float("EI", integralLM, integralRM)
    elif whichUpdate == 6:
        send_float("ED", filtLM_dErr, filtRM_dErr)
    elif whichUpdate == 7:
        send_int("MO", outLM, outRM)
    elif whichUpdate == 8:
        print("WP %d" % wp_state)
    elif whichUpdate == 9:
        print("GO %d" % ctrlEnabled)
    elif whichUpdate in (10, 11, 12, 13, 14):
        send_ultrasonic(whichUpdate - 10)
    elif whichUpdate == 15:
        if whichControlUpdate == 1:
            send_float("Kp", LM_Kp, RM_Kp)
        elif whichControlUpdate == 2:
            send_float("Ki", LM_Ki, RM_Ki)
        elif whichControlUpdate == 3:
            send_float("Kd", LM_Kd, RM_Kd)
        elif whichControlUpdate == 4:
            send_float("Kf", LM_Kf, RM_Kf)
        elif whichControlUpdate == 5:
            send_float("Ka", LM_Ka, RM_Ka)
        elif whichControlUpdate == 6:
            send_float("Kb", LM_Kb, RM_Kb)
        elif whichControlUpdate == 7:
            send_int("Db", LM_Db, RM_Db)
        elif whichControlUpdate == 8:
            send_int("Mi", int(LM_Mi), int(RM_Mi))
        elif whichControlUpdate == 9:
            send_int("Mj", int(LM_Mj), int(RM_Mj))
        elif whichControlUpdate == 10:
            send_float("Id", LM_Id, RM_Id)
        elif whichControlUpdate == 11:
            send_int("Am", int(LM_Am * cf), int(RM_Am * cf))
        elif whichControlUpdate == 12:
            print("Version %s" % VERSION)
        whichControlUpdate = 0 if whichControlUpdate >= 12 else whichControlUpdate
        whichControlUpdate += 1

    whichUpdate = 0 if whichUpdate >= 15 else whichUpdate
    whichUpdate += 1


def main():
    global TimeSpeedLastCommand, TimePumpLastCommand, ctrlEnabled, wp_state
    global LM_target_setvel, RM_target_setvel

    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    rx_cmd = ""

    last_ctrl_time = time.ticks_ms()
    last_update_time = time.ticks_ms()
    TimeSpeedLastCommand = last_ctrl_time
    TimePumpLastCommand = last_ctrl_time

    while True:
        now = time.ticks_ms()

        while poll.poll(0):
            ch = sys.stdin.read(1)
            if ch == "\n":
                user_command(rx_cmd)
                rx_cmd = ""
            elif len(rx_cmd) < 80:
                rx_cmd += ch

        if time.ticks_diff(now, TimeSpeedLastCommand) > CMD_TIMEOUT_MS:
            LM_target_setvel = 0.0
            RM_target_setvel = 0.0
            lm_set_motor_speed(0)
            rm_set_motor_speed(0)
            ctrlEnabled = 0
            TimeSpeedLastCommand = now

        if time.ticks_diff(now, TimePumpLastCommand) > CMD_TIMEOUT_MS:
            wp_state = 0
            pump1_en.value(0)
            TimePumpLastCommand = now

        time_since_last_ctrl = time.ticks_diff(now, last_ctrl_time)
        if time_since_last_ctrl >= CTRL_LOOP_STEP_MS:
            last_ctrl_time = time.ticks_add(last_ctrl_time, CTRL_LOOP_STEP_MS)
            control_step()
        elif time.ticks_diff(now, last_update_time) >= 10 and time_since_last_ctrl < CTRL_LOOP_STEP_MS - 20:
            last_update_time = time.ticks_add(last_update_time, 10)
            send_telemetry()


main()
