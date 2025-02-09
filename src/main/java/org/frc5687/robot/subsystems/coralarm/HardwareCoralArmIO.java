package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;
import org.frc5687.robot.util.sensors.RevBoreEncoder;

public class HardwareCoralArmIO implements CoralArmIO {
    private final RevBoreEncoder _encoder;
    private final VictorSP _pivotMotor;
    private final VictorSP _wheelMotor;
    private final ProfiledPIDController _controller;
    private final ProximitySensor _coralDetectionSensor;

    public HardwareCoralArmIO() {
        _encoder = new RevBoreEncoder(RobotMap.DIO.CORAL_ENCODER, -1.03);
        _encoder.setInverted(true);
        _pivotMotor = new VictorSP(RobotMap.PWM.CORAL_PIVOT_MOTOR);
        _wheelMotor = new VictorSP(RobotMap.PWM.CORAL_WHEEL_MOTOR);
        _coralDetectionSensor = new ProximitySensor(RobotMap.DIO.CORAL_SENSOR);

        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.CoralArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.CoralArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _controller =
                new ProfiledPIDController(
                        Constants.CoralArm.kP, Constants.CoralArm.kI, Constants.CoralArm.kD, constraints);

        _controller.setTolerance(0.01);
        _pivotMotor.setInverted(Constants.CoralArm.PIVOT_MOTOR_INVERTED);

        _controller.reset(_encoder.getAngle());
    }

    private void calculateShortestPath(double currentAngle) {
        if (currentAngle < Units.degreesToRadians(90) || currentAngle >= Units.degreesToRadians(270)) {
            _controller.enableContinuousInput(0, 2.0 * Math.PI);
        } else {
            _controller.disableContinuousInput();
        }
    }

    private double processSafeAngle(double desiredAngle) {
        return MathUtil.clamp(desiredAngle, Constants.CoralArm.MIN_ANGLE, Constants.CoralArm.MAX_ANGLE);
    }

    private double calculateFeedForward(double angle) {
        return ((Constants.CoralArm.ARM_LENGTH / 2.0)
                        * (Constants.CoralArm.GEARBOX.rOhms * Constants.CoralArm.ARM_MASS * 9.81)
                        / (Constants.CoralArm.GEAR_RATIO * Constants.CoralArm.GEARBOX.KtNMPerAmp))
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
        double currentAngle = _encoder.getAngle();
        double safeAngle = processSafeAngle(outputs.desiredAngleRad);
        calculateShortestPath(currentAngle);

        _controller.setGoal(safeAngle);

        double pidOutput = _controller.calculate(currentAngle);
        double ffOutput = calculateFeedForward(currentAngle);

        double totalVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);

        outputs.voltageCommand = totalVoltage;
        outputs.controllerOutput = pidOutput;
        outputs.voltageFeedForward = ffOutput;

        _pivotMotor.setVoltage(totalVoltage);
        _wheelMotor.setVoltage(outputs.wheelVoltageCommand);
    }
}
