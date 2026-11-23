package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

@Config
public class Intake extends SubsystemBase {

    // Flip this if "intake in" spins the wrong way.
    public static boolean REVERSED = false;
    public static double IN_POWER = 1.0;
    public static double OUT_POWER = -0.6;

    private final DcMotorEx intake;
    private final TelemetryData telemetry;
    private double power = 0.0;

    public Intake(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // The intake runs open loop. Its encoder port is left free so the turret's REV encoder can
        // live there if the turret motor's own port is taken (see Turret.ENCODER_NAME).
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        intake.setPower(power);
        telemetry.addData("Intake Power", power);
    }

    public void in() {
        power = IN_POWER;
    }

    public void out() {
        power = OUT_POWER;
    }

    public void stop() {
        power = 0.0;
    }

    public void setPower(double power) {
        this.power = power;
    }
}
