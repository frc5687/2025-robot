package org.frc5687.robot.commands.coral;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class IntakeCoralWheels extends OutliersCommand {
    private final CoralArmSubsystem _coral;
    private int state;

    public IntakeCoralWheels(RobotContainer container) {
        _coral = container.getCoral();
    }

    @Override
    public void initialize() {
        state = 0;
        _coral.setWheelMotorDutyCycle(0.25);
    }

    @Override
    protected void execute(double timestamp) {
        System.out.println("statte :" + state);
        switch (state) {
            case 0:
                if (_coral.isCoralDetected()) {
                    _coral.setWheelMotorPosition(_coral.getWheelMotorPosition() + 1.5);
                    state = 1;
                }
                break;
            case 1:
                if (_coral.isAtDesiredWheelAngle()) {
                    state = 2;
                }
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == 2;
    }
}
