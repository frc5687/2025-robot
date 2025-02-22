package org.frc5687.robot.commands.coral;

import org.frc5687.robot.Constants;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class EjectCoral extends OutliersCommand {
    private final CoralArmSubsystem _coral;

    public EjectCoral(CoralArmSubsystem coral) {
        _coral = coral;
        addRequirements(_coral);
    }

    @Override
    public void initialize() {
        super.initialize();
        // _coral.setArmAngle(_coral.getArmAngleRads() + Units.degreesToRadians(40));
        _coral.setWheelMotorDutyCycle(Constants.CoralArm.WHEEL_EJECT_CORAL_DUTY_CYCLE);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _coral.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        _coral.mapToClosestState();
        super.end(interrupted);
    }
}
