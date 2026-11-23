# TeamCode: BIOBUZZ (2026-2027) robot

FTC SDK 12.0 · SolversLib 0.3.6 · Pedro Pathing 3.0.2

## Robot configuration (Driver Station → Configure Robot)

| Name        | Device                              | Notes                                             |
|-------------|-------------------------------------|---------------------------------------------------|
| `rb` `rf` `lb` `lf` | drive motors, Control Hub ports 0, 1, 2, 3 | left side reversed (`pedroPathing/Constants`) |
| `pinpoint`  | goBILDA Pinpoint (I2C)              | 4-bar pods                                        |
| `intake`    | intake motor                        | open loop                                         |
| `shooter`   | flywheel motor                      | velocity from its own encoder                     |
| `turret`    | goBILDA 435 rpm, 50T → 100T turret   | Expansion Hub (Control Hub's 4 ports are the drive) |
| (encoder)   | REV Through Bore, 20T on the turret, quadrature (A/B) | plug into the `turret` motor's encoder port, or set `Turret.ENCODER_NAME` to the motor whose port it uses |
| `limelight` | Limelight 3A (Ethernet/USB)         | pipeline 0 = AprilTag, 36h11                      |

## First flash checklist

1. **Turret on its reference position when you power on the robot.** The zero is hardcoded to that
   physical position (`Turret.REFERENCE_DEG`, default 0 = facing forward). No OpMode zeros the turret
   by itself; the hub keeps counting from power-on through auto and teleop, so the turret moving during
   auto can't shift it. INIT shows the turret angle; if it's wrong with the turret on the mark,
   press `Back` during INIT. For the most repeatable zero, power on with the turret against a hard stop
   and set `REFERENCE_DEG` to that stop's angle (e.g. -170).
   Don't plug the Through Bore into the `shooter` port: the shooter resets its own encoder.
2. Run **Mechanism Test**. Check in order:
   - Right stick up turns the turret CCW and `Turret Angle` goes **up**. If not, flip `Turret.MOTOR_REVERSED` / `ENCODER_REVERSED`.
   - If telemetry shows `TURRET STOPPED`, the encoder counts the wrong way: flip `Turret.ENCODER_REVERSED`.
   - Turn the turret one full turn by hand: angle should change by 360 (the encoder turns 5x, 100T/20T).
   - Set `Turret.MIN_DEG` / `MAX_DEG` inside the wiring's hard stops.
   - `X` spins the shooter; `Shooter Actual (RPM)` must be positive. If not, flip `Shooter.REVERSED`.
   - Intake: `RB` should pull a ball in; else flip `Intake.REVERSED`.
3. Calibrate drive with Pedro AutoTune (`pedroPathing/Tuning.java`, open `http://192.168.43.1:10158`),
   and paste the results into `pedroPathing/Constants.java` (Pinpoint offsets, Foresight gains).
4. Measure and fill the `TODO`s in `FieldConstants` (HIVE/CELL positions, up-CELL tag height),
   `Vision` (lens height and tilt) and `AimSolver` (turret pivot offset).
5. Tune live with FTC Dashboard (`http://192.168.43.1:8080/dash`) or Panels:
   - Turret: `kP`, `kD`, `kS` (then `kI` only if it parks short). Use `A` in Mechanism Test.
   - Shooter: `kF` first, then `kP`, then a small `kI` if it settles low.
   - **RPM table**: park at several distances, find the RPM that scores, and enter
     `Shooter.DISTANCE_IN` / `RPM_AT_DISTANCE`. Mechanism Test shows the aim distance.

## Auto-aim

`AimSolver` aims the turret at the up-facing CELL of our HIVE:

- **Tags visible:** the Limelight finds our CELL clusters (red 30-33 far / 34-37 audience,
  blue 42-45 far / 38-41 audience) and picks the higher one, which is the up CELL.
  Each new frame is projected onto the field to learn odometry's error.
- **Tags not visible:** aims from Pinpoint odometry at the CELL position plus the last learned correction.
- **Pinpoint faulted:** aims straight off the camera; with neither available it holds position.

The distance to the target picks the flywheel RPM from the lookup table.

## Driver controls (Red / Blue TeleOp, gamepad 1)

| Input | Action |
|---|---|
| Left stick / right stick X | field-centric drive / turn |
| Right trigger (hold) | shoot: spin up for the distance, auto-feed when at speed and on target |
| RB / LB (hold) | intake in / out |
| Y | auto-aim on/off (off = turret forward) |
| D-pad left/right, down | trim aim 1°, clear trim |
| D-pad up | switch target CELL manually |
| Start | reset pose to the alliance reset spot (`FieldConstants.RESET_*`) |
| Back (INIT only) | re-zero turret (turret must be on its reference position) |

Teleop starts from the pose the last OpMode left (`PoseStorage`), or the alliance reset spot if none ran.
