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
 * The Through Bore's quadrature output is relative, so the turret MUST face straight forward the
 * first time an OpMode runs after the robot powers on. That position becomes 0 and is kept across
 * OpModes (auto -> teleop) until the app restarts. {@link #rezero()} re-zeros on demand.
 */
@Config
public class Turret extends SubsystemBase {

    // --- Hardware ---
    // Motor port whose ENCODER connector the Through Bore is plugged into. Use "turret" if it's on the
    // turret motor's own port; if it's on another motor's port (e.g. the intake, which runs open loop),
    // put that motor's config name here.
    public static String ENCODER_NAME = "turret";
    public static boolean MOTOR_REVERSED = false;   // flip so positive power turns the turret CCW
    public static boolean ENCODER_REVERSED = false; // flip so the angle increases when turning CCW

    // REV Through Bore: 8192 counts per revolution of the encoder.
    public static double ENCODER_CPR = 8192;
    // Encoder revolutions per turret revolution. 1.0 if the encoder sits on the turret axis itself.
    public static double ENCODER_TO_TURRET_RATIO = 1.0; // TODO: verify

    // Soft limits so the turret never winds up its wiring. Keep a margin inside the hard stops.
    public static double MIN_DEG = -160; // TODO: set from the robot
    public static double MAX_DEG = 160;  // TODO: set from the robot

    // --- Position controller (PID + static feedforward) ---
    public static double kP = 0.02;     // TODO: tune
    public static double kI = 0.0;      // only if it parks short of target under load
    public static double kD = 0.0008;   // TODO: tune
    public static double I_ZONE_DEG = 8; // integrate only when this close, so big moves don't wind up
    public static double I_MAX_POWER = 0.1;
    public static double kS = 0.04;     // min power to break static friction
    public static double TOLERANCE_DEG = 1.5;
    public static double MAX_POWER = 0.8;

    // Survives OpMode changes (same as PoseStorage) so auto -> teleop doesn't lose the zero.
    private static boolean sZeroed = false;

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

    public Turret(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        motor = hardwareMap.get(DcMotorEx.class, "turret");
        encoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);

        motor.setDirection(MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (!sZeroed) {
            rezero();
        }
        angleDeg = readAngle();
        targetDeg = angleDeg;
        timer.reset();
    }

    @Override
    public void periodic() {
        angleDeg = readAngle();

        double power;
        if (!enabled) {
            // Manual mode (tuning OpMode): still refuse to drive past the soft limits.
            power = manualPower;
            if ((angleDeg >= MAX_DEG && power > 0) || (angleDeg <= MIN_DEG && power < 0)) {
                power = 0.0;
            }
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

    /** True if {@code deg} had to be clamped, i.e. the turret can't reach it without the drivetrain turning. */
    public boolean isOutOfRange(double deg) {
        return deg < MIN_DEG || deg > MAX_DEG;
    }

    /** Make the current position the new 0 (turret must face robot-forward). */
    public void rezero() {
        // STOP_AND_RESET also stops the motor on that port for one cycle; put it back in open-loop mode
        // straight away so a shared port (e.g. the intake) keeps working.
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sZeroed = true;
        angleDeg = 0.0;
        targetDeg = 0.0;
        lastError = 0.0;
        integral = 0.0;
    }

    private double readAngle() {
        double ticks = encoder.getCurrentPosition() * (ENCODER_REVERSED ? -1 : 1);
        return ticks / (ENCODER_CPR * ENCODER_TO_TURRET_RATIO) * 360.0;
    }

    private static double clamp(double deg) {
        return Math.max(MIN_DEG, Math.min(MAX_DEG, deg));
    }
}
