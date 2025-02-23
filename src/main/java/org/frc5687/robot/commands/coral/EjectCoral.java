package org.frc5687.robot.commands.coral;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class EjectCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;

    public EjectCoral(RobotContainer container) {
        _coral = container.getCoral();
    }

    @Override
    public void initialize() {
        _coral.setWheelMotorDutyCycle(-0.5);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public void end(boolean interrupted) {
        _coral.setWheelMotorPosition(_coral.getWheelMotorPosition());
    }
}
