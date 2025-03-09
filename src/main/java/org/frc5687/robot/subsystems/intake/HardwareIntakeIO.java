package org.frc5687.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.LaserCan;
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
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.TunableDouble;

// import org.frc5687.robot.util.CTREUtil;

public class HardwareIntakeIO implements IntakeIO {

    private final TalonFX _pivotMotor;
    private final TalonFX _rollerMotor;
    private final TalonFX _beltMotor;
    private final CANcoder _encoder;
    private final StatusSignal<Angle> _encoderAngle;
    private final StatusSignal<Angle> _pivotMotorAngle;

    private final VoltageOut _rollerVoltageReq = new VoltageOut(0);
    private final VoltageOut _intakeVoltageReq = new VoltageOut(0);
    private final MotionMagicVoltage _pivotPositionReq;

    private final LaserCan _laserCan;

    private static final TunableDouble detectionRange =
            new TunableDouble("Intake", "detection range", 50);

    public HardwareIntakeIO() {
        _pivotMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ARM, Constants.Intake.CAN_BUS);
        _rollerMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_ROLLER, Constants.Intake.CAN_BUS);
        _beltMotor = new TalonFX(RobotMap.CAN.TALONFX.INTAKE_BELT, Constants.Intake.CAN_BUS);
        _laserCan = new LaserCan(RobotMap.CAN.LASERCAN.INTAKE);
        _encoder = new CANcoder(RobotMap.CAN.CANCODER.INTAKE_ENCODER, Constants.Intake.CAN_BUS);
        configureCancoder();

        _encoderAngle = _encoder.getAbsolutePosition();
        _encoderAngle.refresh();
        double encoderRad = _encoderAngle.getValue().in(Radians);
        _pivotPositionReq = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _pivotMotorAngle = _pivotMotor.getPosition();
        _pivotMotor.setPosition(Units.radiansToRotations(encoderRad * Constants.Intake.GEAR_RATIO));

        configureMotor(_rollerMotor, Constants.Intake.ROLLER_INVERTED, false);
        configureMotor(_beltMotor, Constants.Intake.INTAKE_INVERTED, false);
        configureMotor(_pivotMotor, Constants.Intake.PIVOT_INVERTED, true);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        StatusSignal.refreshAll(_pivotMotorAngle, _encoderAngle);
        var measurement = _laserCan.getMeasurement();
        if (measurement == null) {
            System.err.println("Intake laserCan measurement was null");
            inputs.isCoralDetected = false;
        } else {
            inputs.isCoralDetected = measurement.distance_mm < detectionRange.get();
        }
        inputs.armAngleRads = _pivotMotorAngle.getValue().in(Radians);
    }

    @Override
    public void writeOutputs(IntakeOutputs outputs) {
        _rollerMotor.setControl(_rollerVoltageReq.withOutput(outputs.rollerVoltage));
        _beltMotor.setControl(_intakeVoltageReq.withOutput(outputs.intakeVoltage));
        double safeDesiredAngle =
                Math.min(
                        Math.max(outputs.desiredAngleRad, Constants.Intake.MIN_ANGLE),
                        Constants.Intake.MAX_ANGLE);
        _pivotMotor.setControl(
                _pivotPositionReq
                        .withPosition(Units.radiansToRotations(safeDesiredAngle))
                        .withFeedForward(outputs.dynamicsFF));
    }

    private void configureMotor(TalonFX motor, boolean isInverted, boolean attachCANcoder) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        config.MotionMagic.MotionMagicCruiseVelocity = Constants.Intake.MAX_VELOCITY_RAD_PER_SEC;
        config.MotionMagic.MotionMagicAcceleration =
                Constants.Intake.MAX_ACCELERATION_RAD_PER_SEC_SQUARED;
        config.MotionMagic.MotionMagicJerk = 7000;
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
            config.ClosedLoopGeneral.ContinuousWrap = false;
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                    Units.radiansToRotations(Constants.Intake.MAX_ANGLE * Constants.Intake.GEAR_RATIO);
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        }
        motor.getConfigurator().apply(config);
        // CTREUtil.applyConfiguration(motor, config);
    }

    private void configureCancoder() {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.75));
        _cancoderConfigs.MagnetSensor.MagnetOffset = Constants.Intake.ENCODER_OFFSET;
        _cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        _encoder.getConfigurator().apply(_cancoderConfigs);
    }
}
