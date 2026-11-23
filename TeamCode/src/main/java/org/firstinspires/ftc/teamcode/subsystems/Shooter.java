package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

/**
 * Single-motor flywheel with velocity PIDF and battery-voltage feedforward compensation.
 * Velocity is read from the shooter motor's own encoder.
 */
@Config
public class Shooter extends SubsystemBase {

    // --- Motor constants (verify for your motor) ---
    // goBILDA Yellow Jacket / REV HD Hex bare motor: 28 counts per motor revolution.
    public static double MOTOR_CPR = 28;
    // Flywheel revolutions per motor revolution (1.0 for direct drive).
    public static double GEAR_RATIO = 1.0;
    public static boolean REVERSED = false;

    // --- PIDF (units: motor power per ticks/sec) ---
    // Tune order: kF first (P=I=D=0, raise kF until it holds speed), then kP for recovery after a shot,
    // then a little kI only if it settles below target. All live-editable from FTC Dashboard / Panels.
    public static double kP = 0.008;   // TODO: tune
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double I_MAX_POWER = 0.15; // cap on the integral term's contribution
    public static double kF = 0.0005;  // TODO: tune so kF * target TPS ~= the power that holds speed
    public static double kS = 0.02;    // small constant to overcome friction

    // --- Voltage compensation ---
    public static double VOLTAGE_POLL_HZ = 40;
    public static double NOMINAL_VOLTAGE = 12.4;

    // RPM the shooter is considered "ready" within.
    public static double READY_TOLERANCE_RPM = 75;

    // Lookup table: distance to the goal (inches) -> flywheel RPM. Fill it in from test shots: park at
    // a distance, adjust RPM on the dashboard until shots land, write the pair here. Keep DISTANCE_IN
    // ascending. Values in between are linearly interpolated; outside the table the end value is used.
    public static double[] DISTANCE_IN = {24, 48, 72, 96, 120, 144};          // TODO: tune
    public static double[] RPM_AT_DISTANCE = {2400, 2700, 3000, 3300, 3600, 3900}; // TODO: tune
    public static double DEFAULT_RPM = 3000; // used when the Limelight can't see the goal

    private final DcMotorEx motor;
    private final VoltageSensor voltageSensor;
    private final TelemetryData telemetry;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private double targetTps = 0.0;
    private double currentTps = 0.0;
    private double lastError = 0.0;
    private double integral = 0.0;
    private double cachedVoltage;

    public Shooter(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cachedVoltage = voltageSensor.getVoltage();
        pidTimer.reset();
        voltageTimer.reset();
    }

    @Override
    public void periodic() {
        if (voltageTimer.milliseconds() > 1000.0 / VOLTAGE_POLL_HZ) {
            cachedVoltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }

        currentTps = motor.getVelocity();
        double error = targetTps - currentTps;

        double dt = Math.max(pidTimer.seconds(), 1e-3);
        pidTimer.reset();
        double derivative = (error - lastError) / dt;
        lastError = error;

        double voltageScale = Math.max(0.5, cachedVoltage / NOMINAL_VOLTAGE);

        if (Math.abs(targetTps) < 10) {
            integral = 0.0;
        } else if (kI != 0) {
            integral += error * dt;
            double iLimit = I_MAX_POWER / kI;
            integral = Math.max(-Math.abs(iLimit), Math.min(Math.abs(iLimit), integral));
        }

        double power = kP * error + kI * integral + kD * derivative + (kF / voltageScale) * targetTps + kS;

        if (Math.abs(targetTps) < 10) {
            power = 0.0; // stopped: coast down instead of fighting the flywheel's momentum
        }
        // Never drive the flywheel backwards to slow it down.
        power = Math.max(0.0, Math.min(1.0, power));
        motor.setPower(power);

        telemetry.addData("Shooter Target (RPM)", tpsToRpm(targetTps));
        telemetry.addData("Shooter Actual (RPM)", tpsToRpm(currentTps));
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Battery (V)", cachedVoltage);
    }

    public void setTargetRpm(double rpm) {
        targetTps = rpmToTps(rpm);
    }

    /** Spin to the RPM for a shot from {@code distanceIn} inches. */
    public void setTargetFromDistance(double distanceIn) {
        setTargetRpm(rpmForDistance(distanceIn));
    }

    public void stop() {
        targetTps = 0.0;
    }

    public double getTargetRpm() {
        return tpsToRpm(targetTps);
    }

    public double getRpm() {
        return tpsToRpm(currentTps);
    }

    public boolean isReady() {
        return targetTps > 0 && Math.abs(tpsToRpm(targetTps - currentTps)) <= READY_TOLERANCE_RPM;
    }

    public static double rpmForDistance(double distanceIn) {
        double[] d = DISTANCE_IN;
        double[] r = RPM_AT_DISTANCE;
        int n = Math.min(d.length, r.length);
        if (n == 0) return DEFAULT_RPM;
        if (distanceIn <= d[0]) return r[0];
        for (int i = 1; i < n; i++) {
            if (distanceIn <= d[i]) {
                double t = (distanceIn - d[i - 1]) / (d[i] - d[i - 1]);
                return r[i - 1] + t * (r[i] - r[i - 1]);
            }
        }
        return r[n - 1];
    }

    private static double rpmToTps(double rpm) {
        return rpm / 60.0 * MOTOR_CPR / GEAR_RATIO;
    }

    private static double tpsToRpm(double tps) {
        return tps / (MOTOR_CPR / GEAR_RATIO) * 60.0;
    }
}
