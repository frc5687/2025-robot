package org.frc5687.robot.subsystems.coralarm;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    private final StatusSignal<Angle> _wheelAngle;

    private final DutyCycleOut _wheelDutyCycleOut;
    private final PositionVoltage _wheelPositionController;

    private SimpleMotorFeedforward _ffModel;

    public HardwareCoralArmIO() {
        _cancoder = new CANcoder(RobotMap.CAN.CANCODER.CORAL_ENCODER, "CANivore");

        _pivotMotor = new VictorSP(RobotMap.PWM.CORAL_PIVOT_MOTOR);
        _wheelMotor = new TalonFX(RobotMap.CAN.TALONFX.CORAL_WHEEL_MOTOR, "CANivore");
        _coralDetectionSensor = new ProximitySensor(RobotMap.DIO.CORAL_SENSOR);
        _debouncer = new Debouncer(0.050, Debouncer.DebounceType.kRising);
        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.CoralArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.CoralArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _controller =
                new ProfiledPIDController(
                        Constants.CoralArm.kP, Constants.CoralArm.kI, Constants.CoralArm.kD, constraints);

        _controller.setTolerance(0.01);
        _pivotMotor.setInverted(Constants.CoralArm.PIVOT_MOTOR_INVERTED);

        _ffModel = new SimpleMotorFeedforward(Constants.CoralArm.kS, Constants.CoralArm.kV);
        configureCancoder();
        configureMotor(_wheelMotor, Constants.CoralArm.WHEEL_MOTOR_INVERTED);
        _wheelDutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);
        _wheelPositionController = new PositionVoltage(0).withEnableFOC(true);

        _absoluteAngle = _cancoder.getAbsolutePosition();
        _wheelAngle = _wheelMotor.getPosition();
        _controller.reset(getAngleRads());
        _wheelMotor.setPosition(0);
    }

    // private void calculateShortestPath(double currentAngle) {
    //     if (currentAngle < Units.degreesToRadians(90) || currentAngle >=
    // Units.degreesToRadians(270)) {
    //         _controller.enableContinuousInput(0, 2.0 * Math.PI);
    //     } else {
    //         _controller.disableContinuousInput();
    //     }
    // }

    private double processSafeAngle(double desiredAngle) {
        return MathUtil.clamp(desiredAngle, Constants.CoralArm.MIN_ANGLE, Constants.CoralArm.MAX_ANGLE);
    }

    private double calculateGravityFeedForward(double angle) {
        return ((Constants.CoralArm.ARM_LENGTH / 2.0)
                        * (Constants.CoralArm.GEARBOX.rOhms * Constants.CoralArm.ARM_MASS * 9.81)
                        / (Constants.CoralArm.GEAR_RATIO * Constants.CoralArm.GEARBOX.KtNMPerAmp))
                * Math.cos(angle);
    }

    @Override
    public void updateInputs(CoralInputs inputs) {
        StatusSignal.refreshAll(_absoluteAngle, _wheelAngle);
        inputs.angleRads = getAngleRads();
        inputs.isCoralDetectedRaw = _coralDetectionSensor.get();
        inputs.isCoralDetected = _debouncer.calculate(inputs.isCoralDetectedRaw);
        inputs.wheelAngle = _wheelAngle.getValueAsDouble();
        inputs.motorCurrent = 0.0;
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        double currentAngle = getAngleRads();
        double safeAngle = processSafeAngle(outputs.desiredAngleRad);
        // calculateShortestPath(currentAngle);

        _controller.setGoal(safeAngle);

        double pidOutput = _controller.calculate(currentAngle);
        double dynamicsFF = calculateGravityFeedForward(currentAngle);
        double motorFF = _ffModel.calculate(_controller.getSetpoint().velocity);
        double ffOutput = motorFF + dynamicsFF;

        double totalVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);

        outputs.voltageCommand = totalVoltage;
        outputs.controllerOutput = pidOutput;
        outputs.voltageFeedForward = ffOutput;

        _pivotMotor.setVoltage(totalVoltage);
        if (outputs.wheelPositionControl) {
            _wheelMotor.setControl(_wheelPositionController.withPosition(outputs.wheelPositionCommand));
        } else {
            _wheelMotor.setControl(_wheelDutyCycleOut.withOutput(outputs.wheelVoltageCommand));
        }
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

        config.Slot0.kP = Constants.CoralArm.kP_WHEEL;
        config.Slot0.kI = Constants.CoralArm.kI_WHEEL;
        config.Slot0.kD = Constants.CoralArm.kD_WHEEL;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.CoralArm.WHEEL_CURRENT_LIMIT;

        // motor.getConfigurator().apply(config);
        CTREUtil.applyConfiguration(motor, config);
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {
        _controller.setP(kP);
        _controller.setD(kD);
        _controller.setI(kI);
        _ffModel.setKv(kV);
        _ffModel.setKs(kS);
    }
}
