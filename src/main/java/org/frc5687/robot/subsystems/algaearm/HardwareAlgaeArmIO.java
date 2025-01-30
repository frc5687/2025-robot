package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;

public class HardwareAlgaeArmIO implements AlgaeArmIO {

    private final Encoder _encoder;
    private final VictorSP _pivotMotor;
    private final VictorSP _wheelMotor;
    private final PIDController _pid;
    private final ProximitySensor _algaeDetectionSensor;

    public HardwareAlgaeArmIO() {
        _pid = new PIDController(Constants.AlgaeArm.kP, Constants.AlgaeArm.kI, Constants.AlgaeArm.kD);
        _encoder = new Encoder(RobotMap.DIO.ALGAE_ENCODER_A, RobotMap.DIO.ALGAE_ENCODER_B);
        _pivotMotor = new VictorSP(RobotMap.PWM.ALGAE_PIVOT_MOTOR);
        _wheelMotor = new VictorSP(RobotMap.PWM.ALGAE_WHEEL_MOTOR);
        _algaeDetectionSensor = new ProximitySensor(RobotMap.DIO.ALGAE_SENSOR);
    }

    @Override
    public void updateInputs(AlgaeInputs inputs) {
        inputs.angleRads = _encoder.getDistance();
        inputs.isAlgaeDetected = _algaeDetectionSensor.get();
    }

    @Override
    public void writeOutputs(AlgaeOutputs outputs) {
        double voltage = _pid.calculate(_encoder.getDistance(), outputs.desiredAngleRad);
        _pivotMotor.setVoltage(voltage);
        // _wheelMotor.setVoltage(Outputs.algaeMotorVoltage);
    }
}
