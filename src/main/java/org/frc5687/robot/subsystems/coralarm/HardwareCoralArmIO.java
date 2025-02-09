package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.MathUtil;
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
        _pid = new PIDController(Constants.CoralArm.kP, Constants.CoralArm.kI, Constants.CoralArm.kD);
        _encoder = new RevBoreEncoder(RobotMap.DIO.CORAL_ENCODER, -1.03);
        _encoder.setInverted(true);
        _pivotMotor = new VictorSP(RobotMap.PWM.CORAL_PIVOT_MOTOR);
        _wheelMotor = new VictorSP(RobotMap.PWM.CORAL_WHEEL_MOTOR);
        _coralDetectionSensor = new ProximitySensor(RobotMap.DIO.CORAL_SENSOR);

        _pid.enableContinuousInput(0, 2 * Math.PI);

        _pid.setTolerance(0.01);
        _pivotMotor.setInverted(Constants.CoralArm.PIVOT_MOTOR_INVERTED);
    }

    private double processSafeAngle(double desiredAngle) {
        double mechanicallyLimitedAngle =
                MathUtil.clamp(desiredAngle, Constants.CoralArm.MIN_ANGLE, Constants.CoralArm.MAX_ANGLE);

        return MathUtil.clamp(
                mechanicallyLimitedAngle, Constants.CoralArm.MIN_ANGLE, Constants.CoralArm.MAX_ANGLE);
    }

    private double calculateFeedForward(double angle) {
        return ((Constants.CoralArm.ARM_LENGTH / 2.0) * (Constants.CoralArm.ARM_MASS * 9.81))
                * Math.cos(angle);
    }

    @Override
    public void updateInputs(CoralInputs inputs) {
        inputs.angleRads = _encoder.getAngle();
        inputs.isCoralDetected = _coralDetectionSensor.get();
        inputs.motorCurrent = 0.0;
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        double safeAngle = processSafeAngle(outputs.desiredAngleRad);

        double pidOutput = _pid.calculate(_encoder.getAngle(), safeAngle);

        // double ffOutput = calculateFeedForward(_encoder.getAngle());

        double totalVoltage = MathUtil.clamp(pidOutput, -12, 12);
        outputs.voltageCommand = totalVoltage;
        outputs.controllerOutput = pidOutput;
        _pivotMotor.setVoltage(totalVoltage);
        _wheelMotor.setVoltage(outputs.wheelVoltageCommand);
    }
}
