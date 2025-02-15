package org.frc5687.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.sensors.ProximitySensor;

// import org.frc5687.robot.util.CTREUtil;

public class HardwareIntakeIO implements IntakeIO {

    private final TalonFX _pivotMotor;
    private final TalonFX _rollerMotor;
    private final TalonFX _beltMotor;

    private final CANcoder _encoder;
    private final StatusSignal<AngularVelocity> _rollerVelocity;
    private final StatusSignal<AngularVelocity> _intakeVelocity;
    private final StatusSignal<Angle> _pivotMotorAngle;
    private final StatusSignal<Angle> _encoderAngle;
    private final VoltageOut _rollerVoltageReq = new VoltageOut(0);
    private final VoltageOut _intakeVoltageReq = new VoltageOut(0);
    private final MotionMagicVoltage _pivotPositionReq;
    private final ProximitySensor _coralDetectionSensor;

    public HardwareIntakeIO() {
        _pivotMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ARM, Constants.Intake.CAN_BUS);
        _rollerMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ROLLER, Constants.Intake.CAN_BUS);
        _beltMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_BELT, Constants.Intake.CAN_BUS);
        _coralDetectionSensor = new ProximitySensor(RobotMap.DIO.INTAKE_SENSOR);
        _encoder = new CANcoder(RobotMap.CAN.CANCODER.INTAKE_ENCODER, Constants.Intake.CAN_BUS);
        configureCancoder();
        _encoderAngle = _encoder.getAbsolutePosition();
        _encoderAngle.refresh();
        double encoderRad = _encoderAngle.getValue().in(Radians);
        _rollerVelocity = _rollerMotor.getVelocity();
        _intakeVelocity = _beltMotor.getVelocity();
        _pivotPositionReq = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _pivotMotorAngle = _pivotMotor.getPosition();
        _pivotMotor.setPosition(Units.radiansToRotations(encoderRad * Constants.Intake.GEAR_RATIO));
        configureMotor(_rollerMotor, Constants.Intake.ROLLER_INVERTED, false);
        configureMotor(_beltMotor, Constants.Intake.INTAKE_INVERTED, false);
        configureMotor(_pivotMotor, Constants.Intake.PIVOT_INVERTED, false); // true
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        StatusSignal.refreshAll(_rollerVelocity, _intakeVelocity, _pivotMotorAngle, _encoderAngle);
        inputs.isCoralDetected = _coralDetectionSensor.get();
        inputs.absoluteEncoderAngleRads = _encoderAngle.getValue().in(Radians);

        inputs.armAngleRads = _pivotMotorAngle.getValue().div(Constants.Intake.GEAR_RATIO).in(Radians);
        // level
        // System.out.println(_armAngle.getValueAsDouble());
        inputs.rollerVelocityRadperSec = Units.rotationsToRadians(_rollerVelocity.getValueAsDouble());
        inputs.rollerTemperatureCelsius = _rollerMotor.getDeviceTemp().getValueAsDouble();

        inputs.angularVelocityRadPerSec = Units.rotationsToRadians(_intakeVelocity.getValueAsDouble());
        inputs.intakeTemperatureCelsius = _beltMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void writeOutputs(IntakeOutputs Outputs) {

        _rollerMotor.setControl(_rollerVoltageReq.withOutput(Outputs.rollerVoltage));
        _beltMotor.setControl(_intakeVoltageReq.withOutput(Outputs.intakeVoltage));
        _pivotMotor.setControl(
                _pivotPositionReq.withPosition(
                        Units.radiansToRotations(Outputs.desiredAngleRad) * Constants.Intake.GEAR_RATIO));
    }

    private void configureMotor(TalonFX motor, boolean isInverted, boolean attachCANcoder) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        config.MotionMagic.MotionMagicCruiseVelocity = 110;
        config.MotionMagic.MotionMagicAcceleration = 600;
        config.MotionMagic.MotionMagicJerk = 700;
        config.Slot0.kP = Constants.Intake.kP;
        config.Slot0.kI = Constants.Intake.kI;
        config.Slot0.kD = Constants.Intake.kD;
        config.Slot0.kS = Constants.Intake.kS;
        config.Slot0.kV = Constants.Intake.kV;
        config.Slot0.kA = Constants.Intake.kA;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Intake.CURRENT_LIMIT;

        if (attachCANcoder) {
            config.Feedback.FeedbackRemoteSensorID = _encoder.getDeviceID();
            config.Feedback.RotorToSensorRatio = Constants.Intake.GEAR_RATIO;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            config.ClosedLoopGeneral.ContinuousWrap = true;
        }
        motor.getConfigurator().apply(config);
        // CTREUtil.applyConfiguration(motor, config);
    }

    private void configureCancoder() {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        _cancoderConfigs.MagnetSensor.MagnetOffset = Constants.Intake.ENCODER_OFFSET;
        _cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        _encoder.getConfigurator().apply(_cancoderConfigs);
    }
}
