package org.frc5687.robot.subsystems;

import org.frc5687.robot.util.BaseInputs;
import org.frc5687.robot.util.BaseOutputs;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoTest extends SubsystemBase{
    Servo _servoTest;
    
    public ServoTest() {
        _servoTest = new Servo(0);
    }

    public void setSpeed(double speed) {
        _servoTest.setSpeed(speed);
    }
}
