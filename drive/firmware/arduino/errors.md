# Known errors in GR6_motor.ino

Found while porting to the Pico (`drive/firmware/pico/main.py`, see its
NOTES.md). Fixed in the Pico port; deliberately left as-is here since
this file is legacy/reference only now that the Pico board is the
replacement - not worth the risk of touching working, deployed
Arduino firmware for bugs that are currently harmless.

## Two copy/paste bugs: right motor using left motor's constant

Both currently harmless because `LM_Db`/`RM_Db` and `LM_Ka`/`RM_Ka`
default to equal values - they would only bite if the left and right
motors were ever tuned with different gains.

1. `RM_setMotorSpeed()` deadbands against `LM_Db` instead of `RM_Db`:

   ```
   void RM_setMotorSpeed(int motorSpeed)
   {
     int absMotorSpeed = abs(motorSpeed);
     if( absMotorSpeed <= LM_Db ) absMotorSpeed = 0;          // Deadband
   ```

2. The right motor's filtered D-term decays with `LM_Ka` instead of
   `RM_Ka` in its second term:

   ```
   filtRM_dErr = filtRM_dErr * RM_Ka + (1.0 - LM_Ka) * dRM;
   ```

## Encoder direction decode - not a bug in this file, but a landmine for reuse

The same/different quadrature decode here (`LM_encoderISR`/
`RM_encoderISR`) works correctly with this board's original SS49E
linear Hall sensors (which produce overlapping, ~50%-duty channels),
but does **not** work with the A3144 unipolar digital Hall switches
used on the new Pico board (narrow, non-overlapping pulses) - see
`drive/firmware/pico/NOTES.md` for the full story and the fix used
there. Not an error in this file as deployed; noted here so the
mismatch isn't rediscovered from scratch if this algorithm is ever
reused elsewhere with different sensor hardware.
