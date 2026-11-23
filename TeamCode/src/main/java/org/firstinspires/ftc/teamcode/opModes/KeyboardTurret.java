package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.KeyboardServer;

/**
 * {@link FieldCentricTurret} driven from a laptop keyboard instead of a gamepad.
 * <ol>
 *   <li>Laptop on the robot's Wi-Fi. Start this OpMode (Driver Hub, or FTC Dashboard's OpMode menu at
 *       http://192.168.43.1:8080/dash).</li>
 *   <li>Open http://192.168.43.1:8099 and keep that tab focused.</li>
 * </ol>
 * W/S forward/back, A/D left/right (field-oriented), J/L or arrow left/right rotate, Shift = fast,
 * Y toggles turret aiming at the field center, R resets the pose to the start spot.
 */
@Config
@TeleOp(name = "Keyboard + Turret", group = "Test")
public class KeyboardTurret extends FieldCentricTurret {

    public static double DRIVE_SPEED = 0.5;
    public static double TURN_SPEED = 0.4;
    public static double FAST_SPEED = 1.0;

    private KeyboardServer keyboard;

    @Override
    public void initialize() {
        super.initialize();
        keyboard = KeyboardServer.get();
        keyboard.setStatus(() -> String.format("Turret: %s\nTurret angle: %.1f deg (target %.1f)\n"
                        + "Pose: %.1f, %.1f, %.1f deg",
                isTracking() ? "AIMING AT CENTER" : "FORWARD",
                turret.getAngle(), turret.getTargetAngle(),
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
    }

    @Override
    public void run() {
        telemetryData.addData("Keyboard", keyboard.isConnected()
                ? "connected" : "open http://192.168.43.1:" + KeyboardServer.PORT);
        super.run();
    }

    private double speed(double normal) {
        return keyboard.isDown("shift") ? FAST_SPEED : normal;
    }

    private double axis(String plus, String minus) {
        return (keyboard.isDown(plus) ? 1 : 0) - (keyboard.isDown(minus) ? 1 : 0);
    }

    @Override
    protected double forwardInput() {
        return axis("w", "s") * speed(DRIVE_SPEED);
    }

    @Override
    protected double leftInput() {
        return axis("a", "d") * speed(DRIVE_SPEED);
    }

    @Override
    protected double turnInput() {
        double ccw = axis("j", "l") + axis("arrowleft", "arrowright");
        return Math.max(-1, Math.min(1, ccw)) * speed(TURN_SPEED);
    }

    @Override
    protected boolean aimButton() {
        return keyboard.isDown("y");
    }

    @Override
    protected boolean resetButton() {
        return keyboard.isDown("r");
    }
}
