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

    private TelemetryData telemetryData;
    private Follower follower;
    private Turret turret;

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
        follower.manual(ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                headingFromStart));

        // --- Buttons ---
        if (gamepad1.y && !lastY) {
            tracking = !tracking;
        }
        lastY = gamepad1.y;

        if (gamepad1.start && !lastStart) {
            follower.setPose(startPose());
        }
        lastStart = gamepad1.start;

        // --- Turret ---
        Pose pose = follower.pose();
        double turretTarget = 0.0;
        if (tracking) {
            double bearing = Math.atan2(TARGET_Y - pose.y(), TARGET_X - pose.x());
            turretTarget = Math.toDegrees(normalize(bearing - pose.heading()));
        }
        turret.setTargetAngle(turretTarget);

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

    private static Pose startPose() {
        return new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
    }

    private static double normalize(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
