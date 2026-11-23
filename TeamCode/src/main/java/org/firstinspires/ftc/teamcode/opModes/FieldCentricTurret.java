package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Field-oriented drive + turret that locks onto the center of the field.
 * <p>
 * Setup: place the robot at (START_X, START_Y) with the robot AND the turret facing forward, then INIT.
 * Default start = middle of a wall, facing the field center, so the turret's first lock is straight ahead.
 * <pre>
 *   Left stick       drive, field-oriented: stick up = the direction the robot faced at the start
 *   Right stick X    turn
 *   Y                toggle: turret tracks the field center / turret faces forward
 *   Start            robot is back at the start spot facing forward: reset pose
 * </pre>
 */
@Config
@TeleOp(name = "Field Centric + Turret", group = "Match")
public class FieldCentricTurret extends CommandOpMode {

    // Where the robot starts, Pedro field inches (0..144). Heading 90 = facing +y, toward the far wall.
    public static double START_X = 72;
    public static double START_Y = 9;
    public static double START_HEADING_DEG = 90;

    // The point the turret locks onto.
    public static double TARGET_X = 72;
    public static double TARGET_Y = 72;

    protected TelemetryData telemetryData;
    protected Follower follower;
    protected Turret turret;

    private boolean started = false;
    private boolean tracking = false;
    private boolean lastY = false;
    private boolean lastStart = false;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose());

        turret = new Turret(hardwareMap, telemetryData);
        turret.rezero(); // turret is facing forward at INIT: that is 0
        register(turret);
    }

    @Override
    public void run() {
        if (!started) {
            // Re-seat the start pose once the match starts, in case the Pinpoint was still
            // calibrating when INIT set it.
            follower.setPose(startPose());
            started = true;
        }

        // --- Drive: field-oriented relative to the robot's starting direction ---
        double heading = follower.pose().heading();
        double headingFromStart = heading - Math.toRadians(START_HEADING_DEG);
        follower.manual(ManualDrive.fieldCentric(forwardInput(), leftInput(), turnInput(), headingFromStart));

        // --- Buttons ---
        boolean y = aimButton();
        if (y && !lastY) {
            tracking = !tracking;
        }
        lastY = y;

        boolean reset = resetButton();
        if (reset && !lastStart) {
            follower.setPose(startPose());
        }
        lastStart = reset;

        // --- Turret ---
        Pose pose = follower.pose();
        double turretTarget = 0.0;
        if (tracking) {
            double bearing = Math.atan2(TARGET_Y - pose.y(), TARGET_X - pose.x());
            turretTarget = Math.toDegrees(normalize(bearing - pose.heading()));
        }
        turret.setTargetWrapped(turretTarget);

        super.run(); // turret PID
        follower.update();

        telemetryData.addData("Turret", tracking ? "TRACKING CENTER" : "FORWARD");
        if (tracking && turret.isOutOfRange(turretTarget)) {
            telemetryData.addData("!!", "center is outside turret travel, turn the robot");
        }
        telemetryData.addData("Pose", String.format("%.1f, %.1f, %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetryData.addData("Distance to center (in)", Math.hypot(TARGET_X - pose.x(), TARGET_Y - pose.y()));
        telemetryData.update();
    }

    // --- Inputs (gamepad 1 here; KeyboardTurret overrides these with the browser keyboard) ---

    /** Drive forward (+) / back (-), -1..1. */
    protected double forwardInput() {
        return -gamepad1.left_stick_y;
    }

    /** Drive left (+) / right (-), -1..1. */
    protected double leftInput() {
        return -gamepad1.left_stick_x;
    }

    /** Turn counter-clockwise (+) / clockwise (-), -1..1. */
    protected double turnInput() {
        return -gamepad1.right_stick_x;
    }

    protected boolean aimButton() {
        return gamepad1.y;
    }

    protected boolean resetButton() {
        return gamepad1.start;
    }

    protected boolean isTracking() {
        return tracking;
    }

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private static double normalize(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
