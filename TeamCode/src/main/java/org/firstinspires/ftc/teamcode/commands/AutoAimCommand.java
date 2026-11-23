package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AimSolver;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Default turret command: keeps the turret on the target from {@link AimSolver} (vision when the tags
 * are in view, odometry otherwise). With aiming switched off it parks facing forward.
 */
public class AutoAimCommand extends CommandBase {

    private final Turret turret;
    private final AimSolver aim;
    private final BooleanSupplier aimEnabled;
    private final DoubleSupplier trimDeg;

    /**
     * @param aimEnabled false parks the turret at 0
     * @param trimDeg    driver offset added on top of the solution (CCW positive)
     */
    public AutoAimCommand(Turret turret, AimSolver aim, BooleanSupplier aimEnabled, DoubleSupplier trimDeg) {
        this.turret = turret;
        this.aim = aim;
        this.aimEnabled = aimEnabled;
        this.trimDeg = trimDeg;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (!aimEnabled.getAsBoolean()) {
            turret.setTargetAngle(0.0);
        } else if (aim.hasSolution()) {
            turret.setTargetWrapped(aim.getTurretTargetDeg() + trimDeg.getAsDouble());
        }
        // No solution (no pose and no tags): leave the turret where it was last pointed.
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
