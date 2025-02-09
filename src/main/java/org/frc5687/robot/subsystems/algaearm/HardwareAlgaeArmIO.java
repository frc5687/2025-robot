package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;
import org.frc5687.robot.util.sensors.RevBoreEncoder;

public class HardwareAlgaeArmIO implements AlgaeArmIO {
    private final RevBoreEncoder _encoder;
    private final VictorSP _pivotMotor;
    private final VictorSP _wheelMotor;
    private final ProfiledPIDController _controller;
    private final ProximitySensor _algaeDetectionSensor;

    public HardwareAlgaeArmIO() {
        _encoder = new RevBoreEncoder(RobotMap.DIO.ALGAE_ENCODER, 4.63);
        _encoder.setInverted(true);
        _pivotMotor = new VictorSP(RobotMap.PWM.ALGAE_PIVOT_MOTOR);
        _wheelMotor = new VictorSP(RobotMap.PWM.ALGAE_WHEEL_MOTOR);
        _algaeDetectionSensor = new ProximitySensor(RobotMap.DIO.ALGAE_SENSOR);

        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.AlgaeArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.AlgaeArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _controller =
                new ProfiledPIDController(
                        Constants.AlgaeArm.kP, Constants.AlgaeArm.kI, Constants.AlgaeArm.kD, constraints);

        _encoder.setInverted(Constants.AlgaeArm.PIVOT_ENCODER_INVERTED);
        _controller.setTolerance(0.01);
        _pivotMotor.setInverted(Constants.AlgaeArm.PIVOT_MOTOR_INVERTED);

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
        return MathUtil.clamp(desiredAngle, Constants.AlgaeArm.MIN_ANGLE, Constants.AlgaeArm.MAX_ANGLE);
    }

    private double calculateFeedForward(double angle) {
        return ((Constants.AlgaeArm.ARM_LENGTH / 2.0)
                        * (Constants.AlgaeArm.GEARBOX.rOhms * Constants.AlgaeArm.ARM_MASS * 9.81)
                        / (Constants.AlgaeArm.GEAR_RATIO * Constants.AlgaeArm.GEARBOX.KtNMPerAmp))
                * Math.cos(angle);
    }

    @Override
    public void updateInputs(AlgaeInputs inputs) {
        inputs.angleRads = _encoder.getAngle();
        inputs.isAlgaeDetected = _algaeDetectionSensor.get();
        inputs.motorCurrent = 0.0;
    }

    @Override
    public void writeOutputs(AlgaeOutputs outputs) {
        double currentAngle = _encoder.getAngle();
        double safeAngle = processSafeAngle(outputs.desiredAngleRad);
        calculateShortestPath(currentAngle);

        _controller.setGoal(safeAngle);

        double pidOutput = _controller.calculate(currentAngle);
        double ffOutput = calculateFeedForward(currentAngle);
        System.out.println(ffOutput);
        double totalVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);

        outputs.voltageCommand = totalVoltage;
        outputs.controllerOutput = pidOutput;

        _pivotMotor.setVoltage(totalVoltage);
        _wheelMotor.setVoltage(outputs.wheelVoltageCommand);
    }
}
