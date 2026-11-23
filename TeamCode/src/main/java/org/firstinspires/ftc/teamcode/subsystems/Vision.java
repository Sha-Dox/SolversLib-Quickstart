package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.FieldConstants.Cell;

import java.util.List;

/**
 * Limelight 3A tracking of our alliance's CELL AprilTag clusters.
 * <p>
 * Each CELL has 4 tags; we average the visible ones into one cluster reading. If both of our CELLS are
 * in view, the one higher in the image (larger ty) is the up-facing CELL, which is the one we shoot at.
 * <p>
 * The Limelight pipeline must be an AprilTag pipeline using the 36h11 family.
 */
@Config
public class Vision extends SubsystemBase {

    // --- Mounting geometry (measure on the robot) ---
    public static double CAMERA_LENS_HEIGHT_IN = 14.0;   // TODO: floor -> lens center
    public static double CAMERA_MOUNT_ANGLE_DEG = 30.0;  // TODO: lens tilt up from horizontal

    public static int PIPELINE = 0;

    // A game piece passing in front of the lens blocks the tag for a few frames. Keep the last good
    // reading for this long so the turret and shooter don't drop out mid-shot.
    public static long TARGET_MEMORY_MS = 250;

    private final Limelight3A limelight;
    private final TelemetryData telemetry;

    private Alliance alliance = Alliance.RED;

    private double tx = 0.0;
    private double ty = 0.0;
    private double distanceIn = 0.0;
    private Cell cell = null;
    private boolean visibleNow = false;
    private boolean newFrame = false;
    private long lastFrameStamp = -1;
    private long lastSeenMs = 0;

    public Vision(HardwareMap hardwareMap, TelemetryData telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        // The loop often runs faster than the camera, so the same frame comes back more than once.
        // The aim solver should only learn from genuinely new frames.
        newFrame = false;
        if (result != null) {
            long stamp = result.getControlHubTimeStamp();
            newFrame = stamp != lastFrameStamp;
            lastFrameStamp = stamp;
        }

        // Average tx/ty per CELL over the visible tags of that CELL's cluster.
        double[] sumTx = new double[Cell.values().length];
        double[] sumTy = new double[Cell.values().length];
        int[] count = new int[Cell.values().length];

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult tag : fiducials) {
                Cell c = FieldConstants.cellOfTag(alliance, tag.getFiducialId());
                if (c == null) continue;
                sumTx[c.ordinal()] += tag.getTargetXDegrees();
                sumTy[c.ordinal()] += tag.getTargetYDegrees();
                count[c.ordinal()]++;
            }
        }

        Cell best = null;
        double bestTy = -Double.MAX_VALUE;
        for (Cell c : Cell.values()) {
            if (count[c.ordinal()] == 0) continue;
            double avgTy = sumTy[c.ordinal()] / count[c.ordinal()];
            if (avgTy > bestTy) {
                bestTy = avgTy;
                best = c;
            }
        }

        visibleNow = best != null;
        if (visibleNow) {
            lastSeenMs = System.currentTimeMillis();
            cell = best;
            tx = sumTx[best.ordinal()] / count[best.ordinal()];
            ty = bestTy;
            double angle = Math.toRadians(CAMERA_MOUNT_ANGLE_DEG + ty);
            double rise = FieldConstants.UP_CELL_TAG_HEIGHT_IN - CAMERA_LENS_HEIGHT_IN;
            distanceIn = angle > 0.01 ? rise / Math.tan(angle) : 0.0;
        } else if (!hasTarget()) {
            // Lost for longer than the memory window: clear so nothing acts on a stale reading.
            tx = 0.0;
            ty = 0.0;
            distanceIn = 0.0;
        }

        if (hasTarget()) {
            telemetry.addData("Tag", (visibleNow ? "LOCKED " : "CACHED ") + cell);
            telemetry.addData("tx / ty", String.format("%.1f / %.1f", tx, ty));
            telemetry.addData("Tag Dist (in)", distanceIn);
        } else {
            telemetry.addData("Tag", "SEARCHING");
        }
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    /** True if a CELL is visible now or was within {@link #TARGET_MEMORY_MS}. */
    public boolean hasTarget() {
        return visibleNow || System.currentTimeMillis() - lastSeenMs < TARGET_MEMORY_MS;
    }

    /** True if a CELL is visible in a frame that arrived this loop (not one already seen). */
    public boolean hasNewTargetFrame() {
        return visibleNow && newFrame;
    }

    /** The CELL last seen as up-facing, or null if none seen yet. */
    public Cell getCell() {
        return cell;
    }

    /** Horizontal angle from the crosshair to the cluster, degrees. Positive = target is to the right. */
    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    /** Horizontal distance from the camera to the up CELL's tag cluster, inches. */
    public double getDistanceInches() {
        return distanceIn;
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public void stop() {
        limelight.stop();
    }
}
