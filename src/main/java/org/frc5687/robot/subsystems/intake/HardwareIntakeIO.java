package org.frc5687.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;

public class HardwareIntakeIO implements IntakeIO {
    private final TalonFX _pivotMotor;
    // private final TalonFX _rollerMotor;
    private final TalonFX _beltMotor;
    private final CANcoder _encoder;
    private final CANrange _range;

    private final StatusSignal<Angle> _encoderAngle;
    private final StatusSignal<Angle> _pivotMotorAngle;
    private final StatusSignal<Boolean> _isCoralDetected;
    private final StatusSignal<Current> _rollerCurrent;

    // private final VoltageOut _rollerVoltageReq = new VoltageOut(0);
    private final VoltageOut _intakeVoltageReq = new VoltageOut(0);
    private double _currentArmAngleRads = 0.0;

    private final MotionMagicVoltage _motionMagicReq;

    public HardwareIntakeIO() {
        _pivotMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ARM, Constants.Intake.CAN_BUS);
        // _rollerMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ROLLER, Constants.Intake.CAN_BUS);
        _beltMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_BELT, Constants.Intake.CAN_BUS);
        _encoder = new CANcoder(RobotMap.CAN.CANCODER.INTAKE_ENCODER, Constants.Intake.CAN_BUS);
        _range = new CANrange(RobotMap.CAN.CANRANGE.INTAKE_CORAL, Constants.Intake.CAN_BUS);
        configureCancoder();
        configureCanrange(_range);

        _isCoralDetected = _range.getIsDetected();

        _rollerCurrent = _beltMotor.getSupplyCurrent();
        _encoderAngle = _encoder.getAbsolutePosition();
        _encoderAngle.refresh();
        double encoderRad = _encoderAngle.getValue().in(Radians);

        _motionMagicReq = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        _pivotMotorAngle = _pivotMotor.getPosition();
        _pivotMotor.setPosition(Units.radiansToRotations(encoderRad * Constants.Intake.GEAR_RATIO));

        // configureMotor(_rollerMotor, Constants.Intake.ROLLER_INVERTED, false);
        configureMotor(_beltMotor, Constants.Intake.INTAKE_INVERTED, false);
        configureMotor(_pivotMotor, Constants.Intake.PIVOT_INVERTED, true);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        StatusSignal.refreshAll(_pivotMotorAngle, _encoderAngle, _isCoralDetected, _rollerCurrent);

        inputs.isCoralDetected = _isCoralDetected.getValue();

        inputs.armAngleRads = _pivotMotorAngle.getValue().in(Radians);
        inputs.encoderAngleRads = _encoderAngle.getValue().in(Radians);
        inputs.rollerCurrent = _rollerCurrent.getValueAsDouble();
        inputs.beltVelocity = _beltMotor.getVelocity().getValueAsDouble();
        _currentArmAngleRads = inputs.armAngleRads;
    }

    @Override
    public void writeOutputs(IntakeOutputs outputs) {
        _beltMotor.setControl(_intakeVoltageReq.withOutput(outputs.intakeVoltage));

        boolean isDesiredZero = Math.abs(outputs.desiredAngleRad) < 0.01;
        boolean isArmNearZero = Math.abs(_currentArmAngleRads) < Math.toRadians(2.0);

        double safeDesiredAngle =
                MathUtil.clamp(
                        outputs.desiredAngleRad, Constants.Intake.MIN_ANGLE, Constants.Intake.MAX_ANGLE);

        double desiredRotations = Units.radiansToRotations(safeDesiredAngle);

        if (isDesiredZero && isArmNearZero) {

            _pivotMotor.setControl(new VoltageOut(0));
        } else {
            _pivotMotor.setControl(_motionMagicReq.withPosition(desiredRotations));

            // double desiredRotations = Units.radiansToRotations(safeDesiredAngle);
            // _pivotMotor.setControl(_motionMagicReq.withPosition(desiredRotations));
        }
    }

    // @Override
    // public void writeOutputs(IntakeOutputs outputs) {
    //     // _rollerMotor.setControl(_rollerVoltageReq.withOutput(outputs.rollerVoltage));
    //     _beltMotor.setControl(_intakeVoltageReq.withOutput(outputs.intakeVoltage));

    //     double safeDesiredAngle =
    //             MathUtil.clamp(
    //                     outputs.desiredAngleRad, Constants.Intake.MIN_ANGLE,
    // Constants.Intake.MAX_ANGLE);

    //     double desiredRotations = Units.radiansToRotations(safeDesiredAngle);
    //     _pivotMotor.setControl(_motionMagicReq.withPosition(desiredRotations));
    // }

    private void configureMotor(TalonFX motor, boolean isInverted, boolean attachCANcoder) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.MAX_VELOCITY_RAD_PER_SEC;
        config.MotionMagic.MotionMagicAcceleration =
                Constants.Intake.MAX_ACCELERATION_RAD_PER_SEC_SQUARED;
        config.MotionMagic.MotionMagicJerk = Constants.Intake.MAX_JERK_RAD_PER_SEC_CUBED;

        config.Slot0.kP = Constants.Intake.kP;
        config.Slot0.kI = Constants.Intake.kI;
        config.Slot0.kD = Constants.Intake.kD;
        config.Slot0.kS = Constants.Intake.kS;
        config.Slot0.kV = Constants.Intake.kV;
        config.Slot0.kA = Constants.Intake.kA;
        config.Slot0.kG = Constants.Intake.kG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.CURRENT_LIMIT;

        if (attachCANcoder) {
            config.CurrentLimits.SupplyCurrentLimit =
                    Constants.Intake.CURRENT_LIMIT; // PIVOT MOTOR LARGER CURRENT
            config.Feedback.FeedbackRemoteSensorID = _encoder.getDeviceID();
            config.Feedback.RotorToSensorRatio = Constants.Intake.GEAR_RATIO;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            config.ClosedLoopGeneral.ContinuousWrap = false;
            config.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = false;
            config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = false;
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                    Units.radiansToRotations(Constants.Intake.MAX_ANGLE * Constants.Intake.GEAR_RATIO);
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        }

        motor.getConfigurator().apply(config);
    }

    private void configureCancoder() {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.75));
        _cancoderConfigs.MagnetSensor.MagnetOffset = Constants.Intake.ENCODER_OFFSET;
        _cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        _encoder.getConfigurator().apply(_cancoderConfigs);
    }

    private void configureCanrange(CANrange range) {
        var canrangeConfigs = new CANrangeConfiguration();
        canrangeConfigs.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        canrangeConfigs.FovParams.FOVRangeX = 7; // TODO: tune
        canrangeConfigs.FovParams.FOVRangeY = 7; // TODO: tune
        // canrangeConfigs.ProximityParams.ProximityHysteresis = 0.04;
        canrangeConfigs.ProximityParams.ProximityThreshold = 0.2; // TODO: Tune
        range.getConfigurator().apply(canrangeConfigs);
    }
}
