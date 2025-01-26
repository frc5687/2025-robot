package org.frc5687.robot.subsystems.algaearm;

import org.frc5687.robot.Constants;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class HardwareAlgaeArmIO implements AlgaeArmIO{

    Encoder _encoder;
    VictorSP _pivotMotor;
    VictorSP _wheelMotor;
    PIDController _pid;
    public HardwareAlgaeArmIO(
        
    ){
        
        _pid = new PIDController(Constants.AlgaeArm.kP, Constants.AlgaeArm.kI, Constants.AlgaeArm.kD);
        _encoder = new Encoder(0, 1);
        _pivotMotor = new VictorSP(2);
        _wheelMotor = new VictorSP(3);
    }
    @Override
    public void updateInputs(AlgaeInputs inputs) {
        inputs.algaeEncoderAngle = _encoder.getDistance();
       
    }

    @Override
    public void writeOutputs(AlgaeOutputs Outputs) {
        double voltage = _pid.calculate(_encoder.getDistance(), Outputs.algaeArmTargetAngle);
        _pivotMotor.setVoltage(voltage);
        _wheelMotor.setVoltage(Outputs.algaeMotorVoltage);
        
    }
}
