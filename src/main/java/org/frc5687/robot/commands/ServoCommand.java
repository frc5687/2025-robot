package org.frc5687.robot.commands;

import org.frc5687.robot.subsystems.ServoTest;

public class ServoCommand extends OutliersCommand {
    ServoTest _servoTest;
    double _speed;
    
    public ServoCommand(ServoTest servoTest, double speed) {
        _servoTest = servoTest;
        _speed = speed;
        addRequirements(_servoTest);
    }

    @Override
    protected void execute(double timestamp) {
        _servoTest.setSpeed(_speed);
    }
}
