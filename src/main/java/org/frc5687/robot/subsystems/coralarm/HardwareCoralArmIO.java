package org.frc5687.robot.subsystems.coralarm;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.CTREUtil;
import org.frc5687.robot.util.sensors.ProximitySensor;

public class HardwareCoralArmIO implements CoralArmIO {
    private final CANcoder _cancoder;
    private final VictorSP _pivotMotor;
    private final TalonFX _wheelMotor;
    private final ProfiledPIDController _controller;
    private final ProximitySensor _coralDetectionSensor;
    private final Debouncer _debouncer;

    private final StatusSignal<Angle> _absoluteAngle;

    public HardwareCoralArmIO() {
        _cancoder = new CANcoder(RobotMap.CAN.CANCODER.CORAL_ENCODER, "CANivore");

        _pivotMotor = new VictorSP(RobotMap.PWM.CORAL_PIVOT_MOTOR);
        _wheelMotor = new TalonFX(RobotMap.CAN.TALONFX.CORAL_WHEEL_MOTOR);
        _coralDetectionSensor = new ProximitySensor(RobotMap.DIO.CORAL_SENSOR);
        _debouncer = new Debouncer(.1, Debouncer.DebounceType.kRising);
        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.CoralArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.CoralArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _controller =
                new ProfiledPIDController(
                        Constants.CoralArm.kP, Constants.CoralArm.kI, Constants.CoralArm.kD, constraints);

        _controller.setTolerance(0.01);
        _pivotMotor.setInverted(Constants.CoralArm.PIVOT_MOTOR_INVERTED);


        configureCancoder();
        configureMotor(_wheelMotor, Constants.CoralArm.WHEEL_MOTOR_INVERTED);
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
        StatusSignal.refreshAll(_absoluteAngle);
        inputs.angleRads = getAngleRads();
        inputs.isCoralDetected = _debouncer.calculate(_coralDetectionSensor.get());
        inputs.motorCurrent = 0.0;
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        double currentAngle = getAngleRads();
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

    private double getAngleRads() {
        return _absoluteAngle.getValueAsDouble() * 2.0 * Math.PI;
    }

    private void configureCancoder() {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
        _cancoderConfigs.MagnetSensor.MagnetOffset = Constants.CoralArm.ENCODER_OFFSET;
        _cancoderConfigs.MagnetSensor.SensorDirection =
                SensorDirectionValue.Clockwise_Positive; // FIXME
        _cancoder.getConfigurator().apply(_cancoderConfigs);
        // CTREUtil.applyConfiguration(_cancoder, _cancoderConfigs);
    }

    private void configureMotor(TalonFX motor, boolean isInverted) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));
        double gearRatio = Constants.Elevator.GEAR_RATIO;
        double metersToRotations = (1.0 / (Constants.Elevator.DRUM_RADIUS)) * gearRatio;

        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Elevator.MOTION_MAGIC_EXPO_KA;
        config.MotionMagic.MotionMagicExpo_kV = Constants.Elevator.MOTION_MAGIC_EXPO_KV;

        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;
        config.Slot0.kG = Constants.Elevator.kG;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        // motor.getConfigurator().apply(config);
        CTREUtil.applyConfiguration(motor, config);
    }
}
