package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.ServoTest;

public class IdleCommand extends OutliersCommand {
    ServoTest _servoTest;
    
    public IdleCommand(ServoTest servoTest) {
        _servoTest = servoTest;
        addRequirements(_servoTest);
    }

    @Override
    protected void execute(double timestamp) {
        _servoTest.setSpeed(0.1);
    }
}
