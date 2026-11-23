package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AimSolver;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Bring-up and tuning. Run this first after flashing to check every direction.
 * <pre>
 *   Left stick      robot-centric drive        Right stick X   turn
 *   Right stick Y   turret manual power (stick up should turn it CCW and raise the angle)
 *   A               turret: hold TEST_TURRET_DEG with the PID   B   turret: back to manual
 *   Right bumper    intake in                   Left bumper    intake out
 *   D-pad up/down   shooter test RPM +/- 100    X              shooter on/off at the test RPM
 *   Back            re-zero turret
 * </pre>
 * Shooter / turret PID gains, the RPM table and TEST_* below are all live-editable in FTC Dashboard
 * (http://192.168.43.1:8080/dash) or Panels.
 */
@Config
@TeleOp(name = "Mechanism Test", group = "Test")
public class MechanismTest extends CommandOpMode {

    public static double TEST_TURRET_DEG = 45;
    public static Alliance TEST_ALLIANCE = Alliance.RED;

    private TelemetryData telemetryData;
    private Follower follower;
    private Drive drive;
    private Turret turret;
    private Intake intake;
    private Shooter shooter;
    private Vision vision;
    private AimSolver aim;
    private GamepadEx controller;

    private double testRpm = 3000;
    private boolean shooterOn = false;
    private boolean turretPid = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);
        drive = new Drive(follower, hardwareMap);
        vision = new Vision(hardwareMap, telemetryData);
        turret = new Turret(hardwareMap, telemetryData);
        intake = new Intake(hardwareMap, telemetryData);
        shooter = new Shooter(hardwareMap, telemetryData);
        aim = new AimSolver(drive, vision, turret, telemetryData);
        vision.setAlliance(TEST_ALLIANCE);
        aim.setAlliance(TEST_ALLIANCE);
        register(drive, vision, turret, intake, shooter, aim);

        drive.resetPose(FieldConstants.resetPose(TEST_ALLIANCE));

        controller = new GamepadEx(gamepad1);
        controller.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> turretPid = true);
        controller.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> turretPid = false);
        controller.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> shooterOn = !shooterOn);
        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> testRpm += 100);
        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> testRpm = Math.max(0, testRpm - 100));
        controller.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(turret::rezero);
    }

    @Override
    public void run() {
        controller.readButtons();

        drive.robotCentricDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (turretPid) {
            turret.setTargetAngle(TEST_TURRET_DEG);
        } else {
            turret.setManualPower(-gamepad1.right_stick_y * 0.4);
        }

        if (gamepad1.right_bumper) {
            intake.in();
        } else if (gamepad1.left_bumper) {
            intake.out();
        } else {
            intake.stop();
        }

        if (shooterOn) {
            shooter.setTargetRpm(testRpm);
        } else {
            shooter.stop();
        }

        super.run();
        drive.updateLocalization();
        follower.update();

        telemetryData.addData("Test RPM", testRpm + (shooterOn ? " (ON)" : " (off)"));
        telemetryData.addData("Shooter Ready", shooter.isReady());
        telemetryData.addData("Turret Mode", turretPid ? "PID -> " + TEST_TURRET_DEG : "MANUAL");
        telemetryData.addData("Aim would point turret at", aim.getTurretTargetDeg());
        telemetryData.addData("LUT RPM for aim distance", Shooter.rpmForDistance(aim.getDistanceInches()));
        telemetryData.addData("Pose", String.format("%.1f, %.1f, %.1f deg",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
        telemetryData.addData("Pinpoint", drive.getLocalizationStatus());
        telemetryData.update();
    }
}
