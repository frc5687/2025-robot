package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;
import org.frc5687.robot.util.sensors.RevBoreEncoder;

public class HardwareCoralArmIO implements CoralArmIO {

    private final RevBoreEncoder _encoder;
    private final VictorSP _pivotMotor;
    private final VictorSP _wheelMotor;
    private final PIDController _pid;
    private final ProximitySensor _coralDetectionSensor;

    public HardwareCoralArmIO() {

        _pid = new PIDController(Constants.AlgaeArm.kP, Constants.AlgaeArm.kI, Constants.AlgaeArm.kD);
        _encoder = new RevBoreEncoder(RobotMap.DIO.CORAL_ENCODER, 0);
        _pivotMotor = new VictorSP(RobotMap.PWM.CORAL_PIVOT_MOTOR);
        _wheelMotor = new VictorSP(RobotMap.PWM.CORAL_WHEEL_MOTOR);
        _coralDetectionSensor = new ProximitySensor(RobotMap.DIO.CORAL_SENSOR);
    }

    @Override
    public void updateInputs(CoralInputs inputs) {
        inputs.angleRads = _encoder.getAngle();
        inputs.isCoralDetected = _coralDetectionSensor.get();
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        // double voltage = _pid.calculate(_encoder.getDistance(), outputs.desiredAngleRad);
        // _pivotMotor.setVoltage(voltage);
        _wheelMotor.setVoltage(outputs.wheelVoltageCommand);
        // _wheelMotor.setVoltage(Outputs.algaeMotorVoltage);
    }
}
