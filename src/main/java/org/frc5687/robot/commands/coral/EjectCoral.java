package org.frc5687.robot.commands.coral;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class EjectCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;

    public EjectCoral(CoralArmSubsystem coral) {
        _coral = coral;
    }

    @Override
    public void initialize() {
        _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_EJECT_CORAL_DUTY_CYCLE);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public void end(boolean interrupted) {
        _coral.holdPosition();
    }
}
