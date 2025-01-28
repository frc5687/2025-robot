package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;

public class HardwareCoralArmIO implements CoralArmIO {

    Encoder _encoder;
    VictorSP _pivotMotor;
    VictorSP _wheelMotor;
    PIDController _pid;

    public HardwareCoralArmIO() {

        _pid = new PIDController(Constants.AlgaeArm.kP, Constants.AlgaeArm.kI, Constants.AlgaeArm.kD);
        _encoder = new Encoder(4, 5);
        _pivotMotor = new VictorSP(2);
        _wheelMotor = new VictorSP(3);
    }

    @Override
    public void updateInputs(CoralInputs inputs) {
        inputs.angleRads = _encoder.getDistance();
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        double voltage = _pid.calculate(_encoder.getDistance(), outputs.desiredAngleRad);
        _pivotMotor.setVoltage(voltage);
        // _wheelMotor.setVoltage(Outputs.algaeMotorVoltage);
    }
}
