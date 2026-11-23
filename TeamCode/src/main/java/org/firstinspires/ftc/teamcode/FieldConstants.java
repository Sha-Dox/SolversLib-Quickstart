package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

/**
 * BIOBUZZ (2026-2027) field geometry used for aiming. Source: Competition Manual section 9.
 * <p>
 * The HIVE Structure sits in the center of the field. Each alliance has one HIVE with two CELLS, one
 * toward the audience and one toward the far wall, ~18.8 in apart. The HIVE pivots (axis 43.95 in above
 * the tiles) so exactly one CELL faces up at a time; that is the one we launch into. Every CELL has a
 * 4-tag AprilTag cluster (36h11, 3.25 in) on its underside, facing down at the tiles.
 * <p>
 * Coordinates are Pedro field coordinates in inches: (0,0) is a field corner, the field is 144 x 144,
 * center (72, 72). This file assumes the audience wall is y = 0 and the red alliance wall is x = 0
 * (the manual puts red on the audience's left).
 * TODO: check these against the real field / CAD and adjust if your field setup differs.
 */
@Config
public class FieldConstants {

    public enum Cell {
        AUDIENCE,
        FAR
    }

    // --- AprilTag IDs per CELL (manual 9.9) ---
    public static final int[] RED_FAR_TAGS = {30, 31, 32, 33};
    public static final int[] RED_AUDIENCE_TAGS = {34, 35, 36, 37};
    public static final int[] BLUE_AUDIENCE_TAGS = {38, 39, 40, 41};
    public static final int[] BLUE_FAR_TAGS = {42, 43, 44, 45};

    // --- CELL aim points (center of the CELL opening when it faces up), Pedro inches ---
    public static double HIVE_CENTER_X = 72.0;
    public static double HIVE_CENTER_Y = 72.0;
    // Sideways offset of each alliance's HIVE from the field center line (red toward x = 0).
    public static double HIVE_SIDE_OFFSET = 10.0;  // TODO: measure
    // Each CELL sits this far from its HIVE's pivot, toward the audience or the far wall (18.8 / 2).
    public static double CELL_OFFSET = 9.4;         // TODO: measure (the up CELL swings a bit further out)

    // Height of the up-facing CELL's tag cluster above the tiles, for the Limelight distance estimate.
    public static double UP_CELL_TAG_HEIGHT_IN = 40.0; // TODO: measure on the field

    // Known spot used when no auto ran before teleop, and by the "reset pose" button: robot's back
    // flat against the audience wall, centered on its alliance's middle tile column, facing the far wall.
    // TODO: pick a spot your drivers can square up on reliably and measure it.
    public static double RESET_X_RED = 36.0;
    public static double RESET_X_BLUE = 108.0;
    public static double RESET_Y = 9.0; // half the robot length off the wall
    public static double RESET_HEADING_DEG = 90.0;

    public static Pose resetPose(Alliance alliance) {
        return new Pose(alliance == Alliance.RED ? RESET_X_RED : RESET_X_BLUE, RESET_Y,
                Math.toRadians(RESET_HEADING_DEG));
    }

    public static int[] tagsFor(Alliance alliance, Cell cell) {
        if (alliance == Alliance.RED) {
            return cell == Cell.FAR ? RED_FAR_TAGS : RED_AUDIENCE_TAGS;
        }
        return cell == Cell.FAR ? BLUE_FAR_TAGS : BLUE_AUDIENCE_TAGS;
    }

    public static double cellX(Alliance alliance) {
        return HIVE_CENTER_X + (alliance == Alliance.RED ? -HIVE_SIDE_OFFSET : HIVE_SIDE_OFFSET);
    }

    public static double cellY(Cell cell) {
        return HIVE_CENTER_Y + (cell == Cell.FAR ? CELL_OFFSET : -CELL_OFFSET);
    }

    /** Which of our CELLS a tag belongs to, or null if it isn't one of ours. */
    public static Cell cellOfTag(Alliance alliance, int id) {
        for (Cell cell : Cell.values()) {
            for (int tag : tagsFor(alliance, cell)) {
                if (tag == id) return cell;
            }
        }
        return null;
    }
}
