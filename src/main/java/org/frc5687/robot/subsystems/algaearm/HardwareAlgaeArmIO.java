package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;
import org.frc5687.robot.util.sensors.RevBoreEncoder;

public class HardwareAlgaeArmIO implements AlgaeArmIO {
    private final RevBoreEncoder _encoder;
    private final VictorSP _pivotMotor;
    private final VictorSP _wheelMotor;
    private final PIDController _pid;
    private final ProximitySensor _algaeDetectionSensor;

    public HardwareAlgaeArmIO() {
        _pid = new PIDController(Constants.AlgaeArm.kP, Constants.AlgaeArm.kI, Constants.AlgaeArm.kD);
        _encoder = new RevBoreEncoder(RobotMap.DIO.ALGAE_ENCODER, 0);
        _encoder.setInverted(true);
        _pivotMotor = new VictorSP(RobotMap.PWM.ALGAE_PIVOT_MOTOR);
        _wheelMotor = new VictorSP(RobotMap.PWM.ALGAE_WHEEL_MOTOR);
        _algaeDetectionSensor = new ProximitySensor(RobotMap.DIO.ALGAE_SENSOR);
        _pid.enableContinuousInput(0, 2 * Math.PI);
        _pid.setTolerance(0.01);
        
        _pivotMotor.setInverted(Constants.AlgaeArm.PIVOT_MOTOR_INVERTED);
    }

    private double processSafeAngle(double desiredAngle) {
        return MathUtil.clamp(desiredAngle, Constants.AlgaeArm.MIN_ANGLE, Constants.AlgaeArm.MAX_ANGLE);
    }

    private double calculateFeedForward(double angle) {
        return (Constants.AlgaeArm.ARM_LENGTH / 2.0) * (Constants.AlgaeArm.ARM_MASS * 9.81) * Math.cos(angle);
    }

    @Override
    public void updateInputs(AlgaeInputs inputs) {
        inputs.angleRads = _encoder.getAngle();
        inputs.isAlgaeDetected = _algaeDetectionSensor.get();
        inputs.motorCurrent = 0.0;
    }

    @Override
    public void writeOutputs(AlgaeOutputs outputs) {
        double safeAngle = processSafeAngle(outputs.desiredAngleRad);
        double pidOutput = _pid.calculate(_encoder.getAngle(), safeAngle);
        double ffOutput = calculateFeedForward(safeAngle); 
        
        double totalVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);
        
        outputs.voltageCommand = totalVoltage;
        outputs.controllerOutput = pidOutput;
        
        _pivotMotor.setVoltage(totalVoltage);
        _wheelMotor.setVoltage(outputs.wheelVoltageCommand);
    }
}