package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.StartEndCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.FieldConstants.Cell;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.commands.AutoAimCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.AimSolver;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/**
 * Driver controls (gamepad 1):
 * <pre>
 *   Left stick        drive (field-centric)       Right stick X   turn
 *   Right trigger     SHOOT: spin up for the target distance, auto-feed once on target
 *   Right bumper      intake in                   Left bumper     intake out
 *   Y                 toggle turret auto-aim (off = turret faces forward)
 *   D-pad left/right  nudge aim trim 1 deg        D-pad down      clear trim
 *   D-pad up          switch target CELL (audience / far) until the camera sees which one is up
 *   Start             reset pose to the alliance reset spot (see FieldConstants)
 *   Back (INIT only)  re-zero turret: only with the turret at its reference position (Turret docs).
 *                     Disabled after Start so a stray press can't shift the zero mid-match.
 * </pre>
 */
public abstract class BaseTeleOp extends CommandOpMode {

    protected final Alliance alliance;

    protected TelemetryData telemetryData;
    protected Follower follower;
    protected Drive mDrive;
    protected Intake mIntake;
    protected Shooter mShooter;
    protected Turret mTurret;
    protected Vision mVision;
    protected AimSolver mAim;
    protected GamepadEx controller;

    private boolean aimEnabled = true;
    private double trimDeg = 0.0;
    private boolean backWasPressed = false;

    public BaseTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);

        mDrive = new Drive(follower, hardwareMap);
        mVision = new Vision(hardwareMap, telemetryData);
        mTurret = new Turret(hardwareMap, telemetryData);
        mIntake = new Intake(hardwareMap, telemetryData);
        mShooter = new Shooter(hardwareMap, telemetryData);
        mAim = new AimSolver(mDrive, mVision, mTurret, telemetryData);

        mVision.setAlliance(alliance);
        mAim.setAlliance(alliance);

        // Order matters: AimSolver reads the vision and turret state from this same loop.
        register(mDrive, mVision, mTurret, mIntake, mShooter, mAim);

        // Odometry-based aiming needs the real field pose: take it from auto if one ran, otherwise
        // assume the robot was placed at the alliance reset spot.
        mDrive.resetPose(PoseStorage.hasPose ? PoseStorage.currentPose : FieldConstants.resetPose(alliance));

        controller = new GamepadEx(gamepad1);
        bindButtons();
    }

    @Override
    public void initialize_loop() {
        // Lets the drive team check the turret zero before the match: with the turret on its reference
        // mark this should read Turret.REFERENCE_DEG. If it doesn't, put it on the mark and press Back.
        if (gamepad1.back && !backWasPressed) {
            mTurret.rezero();
        }
        backWasPressed = gamepad1.back;
        telemetry.addData("Turret angle", "%.1f deg (reference %.1f)", mTurret.readAngle(), Turret.REFERENCE_DEG);
        telemetry.addLine("Turret on its reference mark but wrong angle? Press BACK to re-zero.");
        telemetry.update();
    }

    @Override
    public void run() {
        controller.readButtons();
        super.run();

        // Guard localization BEFORE updating the follower so a Pinpoint glitch/reboot is detected
        // and re-synced this same loop instead of corrupting the field-oriented heading.
        mDrive.updateLocalization();
        follower.update();

        telemetryData.addData("Pose", String.format("%.1f, %.1f, %.1f deg",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
        telemetryData.addData("Auto-aim", aimEnabled ? "ON" : "OFF");
        telemetryData.addData("Trim (deg)", trimDeg);
        telemetryData.addData("Pinpoint", mDrive.getLocalizationStatus());
        telemetryData.addData("PP Faults / Resyncs", mDrive.getLocalizationFaultCount() + " / " + mDrive.getLocalizationResyncCount());
        telemetryData.update();
    }

    private void bindButtons() {
        // Driver stations face each other across the field, so blue's "forward" is red's backward.
        double invert = alliance == Alliance.BLUE ? -1 : 1;
        mDrive.setDefaultCommand(new TeleopDriveCommand(mDrive,
                () -> -gamepad1.left_stick_y * invert,
                () -> -gamepad1.left_stick_x * invert,
                () -> -gamepad1.right_stick_x));

        mTurret.setDefaultCommand(new AutoAimCommand(mTurret, mAim, () -> aimEnabled, () -> trimDeg));

        new Trigger(() -> gamepad1.right_trigger > 0.3)
                .whileActiveOnce(new ShootCommand(mShooter, mIntake, mTurret, mAim));

        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new StartEndCommand(mIntake::in, mIntake::stop, mIntake));
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new StartEndCommand(mIntake::out, mIntake::stop, mIntake));

        controller.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> aimEnabled = !aimEnabled);
        controller.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> trimDeg += 1.0);
        controller.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> trimDeg -= 1.0);
        controller.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> trimDeg = 0.0);
        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() ->
                mAim.setCell(mAim.getCell() == Cell.FAR ? Cell.AUDIENCE : Cell.FAR));

        controller.getGamepadButton(GamepadKeys.Button.START).whenPressed(() -> {
            mDrive.resetPose(FieldConstants.resetPose(alliance));
            mAim.clearCorrections();
        });
    }
}
