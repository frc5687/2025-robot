package org.frc5687.robot.commands.coral;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class IdleCoral extends OutliersCommand {

    private final CoralArmSubsystem _arm;
    private boolean swap = false;
    private boolean lock = false;

    public IdleCoral(CoralArmSubsystem arm) {
        _arm = arm;
        addRequirements(_arm);
    }

    @Override
    protected void execute(double timestamp) {
        // every 2 seconds swap
        // if ((int)timestamp % 2 == 0 && !lock) {
        //     _arm.setDesiredAngleRadians(swap ? 0 : Math.PI);
        //     swap = !swap;
        //     lock = true;
        // } else {
        //     lock = (int)timestamp % 2 == 0;
        // }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
