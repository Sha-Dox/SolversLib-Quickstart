package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.FieldConstants.Cell;

/**
 * Works out where the turret should point and how far away the target is.
 * <p>
 * Aiming always goes through odometry: the target is a field point (the up CELL), and every loop we
 * compute the bearing from the turret pivot to it using the Pinpoint pose. The Limelight corrects that
 * field point: whenever a fresh frame shows our tags, we project the vision bearing + distance into the
 * field and learn how far odometry's idea of the CELL is off. That correction is kept when the tags
 * drop out, so the turret stays on target from odometry alone, and drift is soaked up the next time a
 * tag is seen.
 * <p>
 * Register this AFTER Drive, Vision and Turret so it reads their state from the same loop.
 */
@Config
public class AimSolver extends SubsystemBase {

    // Is the Limelight mounted on the turret (rotates with it) or fixed to the chassis?
    public static boolean LIMELIGHT_ON_TURRET = true;
    public static double CAMERA_YAW_DEG = 0.0; // only used if fixed to the chassis: CCW from forward

    // Turret pivot position relative to the robot's tracking center (Pinpoint pod offsets' origin).
    public static double TURRET_OFFSET_FORWARD_IN = 0.0; // TODO: measure
    public static double TURRET_OFFSET_LEFT_IN = 0.0;    // TODO: measure

    // How quickly the vision correction follows new measurements (0..1 per frame).
    public static double CORRECTION_GAIN = 0.3;
    // Ignore a vision fix that disagrees with odometry by more than this (wrong tag, bad frame).
    public static double MAX_CORRECTION_IN = 36.0;

    public enum Source {
        VISION,   // tags in view this loop (or within the occlusion memory)
        ODOMETRY, // tags out of view: pose + last learned correction
        NONE      // no usable pose and no tags: hold the turret
    }

    private final Drive drive;
    private final Vision vision;
    private final Turret turret;
    private final TelemetryData telemetry;

    private Alliance alliance = Alliance.RED;
    private Cell cell = Cell.FAR;
    // Learned (vision - odometry) error of the CELL position, field inches. Shared by both CELLS since
    // it is mostly odometry drift, which doesn't depend on which CELL we look at.
    private double corrX = 0.0;
    private double corrY = 0.0;

    private Source source = Source.NONE;
    private double turretTargetDeg = 0.0;
    private double distanceIn = 0.0;
    private boolean reachable = true;

    public AimSolver(Drive drive, Vision vision, Turret turret, TelemetryData telemetry) {
        this.drive = drive;
        this.vision = vision;
        this.turret = turret;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        Pose pose = drive.getFollower().pose();
        double heading = drive.getSafeHeading();
        boolean poseOk = drive.isLocalizationHealthy();

        double pivotX = pose.x() + TURRET_OFFSET_FORWARD_IN * Math.cos(heading) - TURRET_OFFSET_LEFT_IN * Math.sin(heading);
        double pivotY = pose.y() + TURRET_OFFSET_FORWARD_IN * Math.sin(heading) + TURRET_OFFSET_LEFT_IN * Math.cos(heading);

        // Follow whichever CELL the camera says is up.
        if (vision.hasTarget() && vision.getCell() != null) {
            cell = vision.getCell();
        }

        if (vision.hasNewTargetFrame() && poseOk) {
            double cameraYaw = LIMELIGHT_ON_TURRET ? Math.toRadians(turret.getAngle()) : Math.toRadians(CAMERA_YAW_DEG);
            // tx is positive to the right (clockwise), our angles are CCW positive.
            double bearing = heading + cameraYaw - Math.toRadians(vision.getTx());
            double d = vision.getDistanceInches();
            if (d > 0) {
                double seenX = pivotX + d * Math.cos(bearing);
                double seenY = pivotY + d * Math.sin(bearing);
                double errX = seenX - FieldConstants.cellX(alliance);
                double errY = seenY - FieldConstants.cellY(cell);
                if (Math.hypot(errX, errY) <= MAX_CORRECTION_IN) {
                    corrX += CORRECTION_GAIN * (errX - corrX);
                    corrY += CORRECTION_GAIN * (errY - corrY);
                }
            }
        }

        if (!poseOk && !vision.hasTarget()) {
            source = Source.NONE;
        } else {
            source = vision.hasTarget() ? Source.VISION : Source.ODOMETRY;
        }

        if (!poseOk && vision.hasTarget()) {
            // Pinpoint is glitching but we can see the tags: aim straight off the camera. Only on new
            // frames, otherwise re-applying the same tx to a moving turret would overshoot.
            if (vision.hasNewTargetFrame()) {
                double base = LIMELIGHT_ON_TURRET ? turret.getAngle() : CAMERA_YAW_DEG;
                turretTargetDeg = base - vision.getTx();
                distanceIn = vision.getDistanceInches();
            }
        } else if (source != Source.NONE) {
            double goalX = FieldConstants.cellX(alliance) + corrX;
            double goalY = FieldConstants.cellY(cell) + corrY;
            double bearing = Math.atan2(goalY - pivotY, goalX - pivotX);
            turretTargetDeg = Math.toDegrees(normalize(bearing - heading));
            distanceIn = Math.hypot(goalX - pivotX, goalY - pivotY);
        }
        reachable = !turret.isOutOfRange(turretTargetDeg);

        telemetry.addData("Aim", source + " -> " + cell + (reachable ? "" : " (TURN ROBOT)"));
        telemetry.addData("Aim Dist (in)", distanceIn);
        telemetry.addData("Aim Corr (in)", String.format("%.1f, %.1f", corrX, corrY));
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    /** Manually pick the CELL to aim at (used until the camera sees which one is up). */
    public void setCell(Cell cell) {
        this.cell = cell;
    }

    public Cell getCell() {
        return cell;
    }

    /** Forget learned vision corrections, e.g. after the robot's pose was reset by hand. */
    public void clearCorrections() {
        corrX = 0.0;
        corrY = 0.0;
    }

    public Source getSource() {
        return source;
    }

    public boolean hasSolution() {
        return source != Source.NONE;
    }

    /** Robot-relative turret angle (degrees, CCW positive) that points at the target. */
    public double getTurretTargetDeg() {
        return turretTargetDeg;
    }

    /** False if the target is outside the turret's travel, so the driver needs to rotate the robot. */
    public boolean isReachable() {
        return reachable;
    }

    /** Horizontal distance from the turret to the target, inches. */
    public double getDistanceInches() {
        return distanceIn;
    }

    private static double normalize(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
