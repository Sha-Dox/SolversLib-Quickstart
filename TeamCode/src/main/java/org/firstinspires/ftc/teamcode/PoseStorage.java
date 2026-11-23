package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;

/**
 * Simple static field to store the pose between OpModes.
 * Drive writes the latest good pose every loop, so teleop picks up wherever auto left the robot.
 */
public class PoseStorage {
    public static Pose currentPose = new Pose(0, 0, 0);
    // False until an OpMode has tracked a pose since the app started (no auto run yet).
    public static boolean hasPose = false;
}
