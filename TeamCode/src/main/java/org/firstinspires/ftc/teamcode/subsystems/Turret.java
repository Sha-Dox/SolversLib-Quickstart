package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

/**
 * Rotating turret driven by one motor, with a REV Through Bore Encoder (quadrature, 8192 CPR) on the
 * turret axis for position feedback.
 * <p>
 * Angles are in degrees, 0 = facing robot-forward, positive = counter-clockwise (left), the same
 * convention Pedro uses for heading.
 * <p>
 * <b>Zeroing is hardcoded to a physical reference position, captured at robot power-on.</b>
 * The Through Bore's quadrature output is relative: the hub starts counting from 0 when it powers up and
 * keeps that count across OpModes (the SDK only switches motors to RUN_WITHOUT_ENCODER between
 * OpModes, it never resets the count). So no OpMode zeros the turret by itself: whatever the turret
 * does during auto, teleop still knows exactly where it is. The only ways to re-zero are
 * {@link #rezero()} (Back during INIT, with the turret at the reference) or a hub power cycle.
 * <p>
 * The reference is wherever the turret sits at power-on, and {@link #REFERENCE_DEG} says what angle
 * that is. Leave it 0 and line the turret up with a forward mark, or (more repeatable) push it against
 * a hard stop at power-on and set REFERENCE_DEG to that stop's angle, e.g. -170.
 */
@Config
public class Turret extends SubsystemBase {

    // --- Hardware ---
    // Motor port whose ENCODER connector the Through Bore is plugged into. Use "turret" if it's on the
    // turret motor's own port; if it's on another motor's port (e.g. the intake, which runs open loop),
    // put that motor's config name here.
    public static String ENCODER_NAME = "turret";
    // Gearing: turret ring 100T, driven by a 50T gear on a goBILDA 435 rpm motor, with a 20T gear on the
    // REV Through Bore. Both gears mesh directly with the turret, so the motor and the encoder spin the
    // OPPOSITE way to the turret (turret right/CW -> motor and encoder CCW).
    public static final double MOTOR_TO_TURRET_RATIO = 100.0 / 50.0; // 2 motor revs per turret rev
    public static final double MOTOR_RPM = 435;                      // -> turret max ~217 rpm
    // Encoder revolutions per turret revolution: 100T / 20T.
    public static double ENCODER_TO_TURRET_RATIO = 100.0 / 20.0;

    // Positive power must turn the turret CCW (left). Because of the gear reversal that means the motor
    // spins CW. The SDK's goBILDA motor spins CCW (seen from the shaft end) on positive FORWARD power,
    // so with the motor shaft pointing UP into the turret gear it has to be REVERSE.
    // TODO: confirm in Mechanism Test (stick up -> turret turns left); flip if the motor hangs shaft-down.
    public static boolean MOTOR_REVERSED = true;
    // Flip so the angle increases while the turret turns CCW. Mechanism Test flags a wrong sign.
    public static boolean ENCODER_REVERSED = false;

    // REV Through Bore (quadrature A/B into a motor encoder port): 8192 counts per encoder revolution,
    // so 8192 * 5 = 40960 counts per turret revolution.
    public static double ENCODER_CPR = 8192;

    // Angle of the turret at its power-on reference position (see class docs). Hardcoded on purpose.
    public static double REFERENCE_DEG = 0.0;

    // Soft limits: how far the turret may turn each way from forward, as if it had wiring to protect.
    // With more than 360 deg of total travel every direction is reachable; when tracking pushes past a
    // limit, setTargetWrapped() swings the turret the long way round to the same direction.
    public static double MIN_DEG = -200;
    public static double MAX_DEG = 200;

    // --- Position controller (PID + static feedforward) ---
    public static double kP = 0.02;     // TODO: tune
    public static double kI = 0.0;      // only if it parks short of target under load
    public static double kD = 0.0008;   // TODO: tune
    public static double I_ZONE_DEG = 8; // integrate only when this close, so big moves don't wind up
    public static double I_MAX_POWER = 0.1;
    public static double kS = 0.04;     // min power to break static friction
    public static double TOLERANCE_DEG = 1.5;
    public static double MAX_POWER = 1.0; // full speed, so a wrap-around swing is as quick as possible

    // Runaway guard (see periodic): how far the turret may move against its power before shutting off.
    // Big enough to ignore coasting through a direction change.
    public static double RUNAWAY_DEG = 20.0;
    public static double RUNAWAY_MIN_POWER = 0.3;

    private final DcMotorEx motor;
    private final DcMotorEx encoder;
    private final TelemetryData telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    private double targetDeg = 0.0;
    private double angleDeg = 0.0;
    private double lastError = 0.0;
    private double integral = 0.0;
    private boolean enabled = true;
    private double manualPower = 0.0;
    private double lastAngle = 0.0;
    private double lastPower = 0.0;
    private double wrongWayDeg = 0.0;
    private boolean faulted = false;

    public Turret(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);

        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Deliberately NO encoder reset here: the count since power-on is the source of truth.
        angleDeg = readAngle();
        lastAngle = angleDeg;
        targetDeg = angleDeg;
        timer.reset();
    }

    @Override
    public void periodic() {
        angleDeg = readAngle();

        double power;
        if (!enabled) {
            power = manualPower; // manual mode (tuning OpMode)
        } else {
            double error = targetDeg - angleDeg;
            double dt = Math.max(timer.seconds(), 1e-3);
            timer.reset();
            double derivative = (error - lastError) / dt;
            lastError = error;

            if (kI != 0 && Math.abs(error) < I_ZONE_DEG) {
                integral += error * dt;
                double iLimit = Math.abs(I_MAX_POWER / kI);
                integral = Math.max(-iLimit, Math.min(iLimit, integral));
            } else {
                integral = 0.0;
            }

            if (Math.abs(error) <= TOLERANCE_DEG) {
                power = 0.0;
            } else {
                power = kP * error + kI * integral + kD * derivative;
                power += Math.copySign(kS, power);
            }
        }

        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // Never push further past a soft limit, in any mode.
        if ((angleDeg >= MAX_DEG && power > 0) || (angleDeg <= MIN_DEG && power < 0)) {
            power = 0.0;
        }

        // Runaway guard: with power applied the angle must move the same way. If it keeps moving the
        // other way, the motor and encoder directions disagree and the PID would drive the turret into
        // its wiring, so shut it off until the OpMode restarts.
        double moved = angleDeg - lastAngle;
        lastAngle = angleDeg;
        if (Math.abs(lastPower) > RUNAWAY_MIN_POWER) {
            double progress = moved * Math.signum(lastPower);
            if (progress > 0.2) {
                wrongWayDeg = 0.0;
            } else if (progress < 0) {
                wrongWayDeg -= progress;
            }
        }
        if (wrongWayDeg > RUNAWAY_DEG) {
            faulted = true;
        }
        if (faulted) {
            power = 0.0;
            telemetry.addData("TURRET STOPPED", "motor/encoder disagree: flip Turret.ENCODER_REVERSED");
        }
        lastPower = power;
        motor.setPower(power);

        telemetry.addData("Turret Angle", angleDeg);
        telemetry.addData("Turret Target", targetDeg);
        telemetry.addData("Turret Power", power);
    }

    /** Point the turret at a robot-relative angle (degrees, CCW positive). Clamped to the soft limits. */
    public void setTargetAngle(double deg) {
        enabled = true;
        targetDeg = clamp(deg);
    }

    /**
     * Point the turret in a direction (degrees, CCW positive, any range). Of all equivalent angles
     * (deg + k*360) inside the soft limits, picks the one closest to where the turret is now. So it
     * follows the target smoothly, and only when the target leaves the travel does it unwind the
     * other way round. The 360 - (MAX - MIN) overlap acts as hysteresis, so it doesn't flip back and forth.
     */
    public void setTargetWrapped(double deg) {
        setTargetAngle(wrapTarget(deg));
    }

    /** See {@link #setTargetWrapped}. Falls back to the nearest limit if no equivalent angle fits. */
    public double wrapTarget(double deg) {
        double nearest = deg - 360.0 * Math.round((deg - angleDeg) / 360.0);
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double candidate = nearest + 360.0 * k;
            if (candidate < MIN_DEG || candidate > MAX_DEG) continue;
            if (Double.isNaN(best) || Math.abs(candidate - angleDeg) < Math.abs(best - angleDeg)) {
                best = candidate;
            }
        }
        return Double.isNaN(best) ? clamp(deg) : best;
    }

    /** Raw power control with no position loop (soft limits still apply). */
    public void setManualPower(double power) {
        enabled = false;
        manualPower = power;
    }

    public double getAngle() {
        return angleDeg;
    }

    public double getTargetAngle() {
        return targetDeg;
    }

    public boolean atTarget() {
        return Math.abs(targetDeg - angleDeg) <= TOLERANCE_DEG;
    }

    /** True if no equivalent of {@code deg} (mod 360) is inside the soft limits, so the robot must turn. */
    public boolean isOutOfRange(double deg) {
        double d = deg - 360.0 * Math.floor((deg - MIN_DEG) / 360.0); // smallest equivalent >= MIN_DEG
        return d > MAX_DEG;
    }

    /**
     * The turret is at its reference position right now: reset the count so it reads
     * {@link #REFERENCE_DEG}. Only call this from INIT or a test OpMode, never mid-match.
     */
    public void rezero() {
        // STOP_AND_RESET also stops the motor on that port for one cycle; put it back in open-loop mode
        // straight away so a shared port (e.g. the intake) keeps working.
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleDeg = REFERENCE_DEG;
        targetDeg = REFERENCE_DEG;
        lastAngle = REFERENCE_DEG;
        lastError = 0.0;
        integral = 0.0;
    }

    /** Read the encoder now (also works during INIT, before periodic() runs). */
    public double readAngle() {
        double ticks = encoder.getCurrentPosition() * (ENCODER_REVERSED ? -1 : 1);
        return REFERENCE_DEG + ticks / (ENCODER_CPR * ENCODER_TO_TURRET_RATIO) * 360.0;
    }

    private static double clamp(double deg) {
        return Math.max(MIN_DEG, Math.min(MAX_DEG, deg));
    }
}
