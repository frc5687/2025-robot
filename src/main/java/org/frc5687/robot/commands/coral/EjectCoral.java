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
        double dutyCycle =
                _coral.getArmAngleRads() < Math.PI
                        ? Constants.CoralArm.WHEEL_EJECT_CORAL_DUTY_CYCLE
                        : Constants.CoralArm.WHEEL_EJECT_TROTH_DUTY_CYCLE;
        _coral.setWheelMotorDutyCycle(dutyCycle);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public void end(boolean interrupted) {
        _coral.holdPosition();
    }
}
