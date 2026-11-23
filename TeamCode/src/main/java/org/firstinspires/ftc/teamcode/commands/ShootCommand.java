package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.AimSolver;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * While held: spin the flywheel to the RPM the lookup table gives for the current target distance, and
 * run the intake to feed once the flywheel is at speed and the turret is on target.
 */
public class ShootCommand extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;
    private final Turret turret;
    private final AimSolver aim;

    public ShootCommand(Shooter shooter, Intake intake, Turret turret, AimSolver aim) {
        this.shooter = shooter;
        this.intake = intake;
        this.turret = turret;
        this.aim = aim;
        addRequirements(shooter, intake);
    }

    @Override
    public void execute() {
        if (aim.hasSolution()) {
            shooter.setTargetFromDistance(aim.getDistanceInches());
        } else {
            shooter.setTargetRpm(Shooter.DEFAULT_RPM);
        }

        boolean onTarget = aim.hasSolution() && aim.isReachable() && turret.atTarget();
        if (shooter.isReady() && onTarget) {
            intake.in();
        } else {
            intake.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
