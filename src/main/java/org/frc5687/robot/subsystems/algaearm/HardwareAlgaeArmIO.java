package org.frc5687.robot.subsystems.algaearm;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;

public class HardwareAlgaeArmIO implements AlgaeArmIO {
    private final CANcoder _cancoder;
    private final VictorSP _pivotMotor;
    private final TalonFX _wheelMotor;
    private final ProfiledPIDController _controller;
    private final ProximitySensor _algaeDetectionSensor;
    private final LinearFilter _angularVelocityFilter;
    private static final DCMotor BAG = DCMotor.getBag(1);
    private double _voltageCommand;
    private final StatusSignal<Angle> _absoluteAngle;

    public HardwareAlgaeArmIO() {
        _cancoder = new CANcoder(RobotMap.CAN.CANCODER.ALGAE_ENCODER, "CANivore");
        _pivotMotor = new VictorSP(RobotMap.PWM.ALGAE_PIVOT_MOTOR);
        _wheelMotor = new TalonFX(RobotMap.CAN.TALONFX.ALGAE_WHEEL);
        _algaeDetectionSensor = new ProximitySensor(RobotMap.DIO.ALGAE_SENSOR);

        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.AlgaeArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.AlgaeArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _controller =
                new ProfiledPIDController(
                        Constants.AlgaeArm.kP, Constants.AlgaeArm.kI, Constants.AlgaeArm.kD, constraints);
        _controller.setTolerance(0.01);
        _pivotMotor.setInverted(Constants.AlgaeArm.PIVOT_MOTOR_INVERTED);
        _angularVelocityFilter =
                LinearFilter.singlePoleIIR(
                        Constants.AlgaeArm.FILTER_TIME_CONSTANT, Constants.UPDATE_PERIOD);
        _voltageCommand = 0;

        configureCancoder();
        _absoluteAngle = _cancoder.getAbsolutePosition();
        _controller.reset(getAngleRads());
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
        StatusSignal.refreshAll(_absoluteAngle);
        double radiansPerSecond =
                new Rotation2d(inputs.angleRads)
                        .minus(new Rotation2d(getAngleRads()))
                        .div(Constants.UPDATE_PERIOD)
                        .getRadians();
        inputs.angleRads = getAngleRads();
        inputs.isAlgaeDetected = _algaeDetectionSensor.get();
        inputs.angularVelocityRadPerSec = _angularVelocityFilter.calculate(radiansPerSecond);
        inputs.motorCurrent =
                BAG.getCurrent(
                        /* FIXME should there be a negative here because of encoder flipping? this actually matters i think - xavier */ inputs
                                        .angularVelocityRadPerSec
                                * Constants.AlgaeArm.GEAR_RATIO,
                        _voltageCommand);
        inputs.motorTorque = BAG.getTorque(inputs.motorCurrent);
        inputs.armTorque = inputs.motorTorque * Constants.AlgaeArm.GEAR_RATIO;
        inputs.isEncoderConnected = _cancoder.isConnected();
    }

    @Override
    public void writeOutputs(AlgaeOutputs outputs) {
        double currentAngle = getAngleRads();
        double safeAngle = processSafeAngle(outputs.desiredAngleRad);
        calculateShortestPath(currentAngle);

        _controller.setGoal(safeAngle);

        double pidOutput = _controller.calculate(currentAngle);
        double ffOutput = calculateFeedForward(currentAngle);

        double totalVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);

        outputs.voltageCommand = totalVoltage;
        outputs.controllerOutput = pidOutput;

        _pivotMotor.setVoltage(totalVoltage);
        _voltageCommand = totalVoltage;
        _wheelMotor.setVoltage(outputs.wheelVoltageCommand);
        // _wheelMotor.set(-0.5);
    }

    private double getAngleRads() {
        return _absoluteAngle.getValueAsDouble() * 2.0 * Math.PI;
    }

    private void configureCancoder() {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
        _cancoderConfigs.MagnetSensor.MagnetOffset = Constants.AlgaeArm.ENCODER_OFFSET;
        _cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        _cancoder.getConfigurator().apply(_cancoderConfigs);
        // CTREUtil.applyConfiguration(_cancoder, _cancoderConfigs);
    }
}
