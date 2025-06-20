package org.frc5687.robot.commands.coral;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class EmergencyEjectCoral extends OutliersCommand {

    private final CoralArmSubsystem _coral;

    public EmergencyEjectCoral(CoralArmSubsystem coral) {
        _coral = coral;
        addRequirements(_coral);
    }

    @Override
    protected void execute(double timestamp) {
        _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_EJECT_CORAL_DUTY_CYCLE);
    }
}
