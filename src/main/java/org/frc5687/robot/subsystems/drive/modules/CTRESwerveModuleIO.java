package org.frc5687.robot.subsystems.drive.modules;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.util.CTREUtil;

public class CTRESwerveModuleIO implements SwerveModuleIO {
    private final TalonFX _driveMotor;
    private final TalonFX _steerMotor;
    private final CANcoder _cancoder;

    private final StatusSignal<Angle> _drivePosition;
    private final StatusSignal<AngularVelocity> _driveVelocity;
    private final StatusSignal<Angle> _steerPosition;
    private final StatusSignal<AngularVelocity> _steerVelocity;

    private final PositionTorqueCurrentFOC _steerMotionMagicReq =
            new PositionTorqueCurrentFOC(0).withUpdateFreqHz(1000.0).withSlot(0);
    private final VelocityTorqueCurrentFOC _driveVelocityReq =
            new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(1000.0);
    private final VoltageOut _driveVoltageReq = new VoltageOut(0);

    private final double _driveRotationsPerMeter;
    private final double _couplingRatio;

    public CTRESwerveModuleIO(
            SwerveModuleConfig config,
            int driveMotorId,
            int steerMotorId,
            int cancoderId,
            String canbus) {
        _driveMotor = new TalonFX(driveMotorId, canbus);
        _steerMotor = new TalonFX(steerMotorId, canbus);
        _cancoder = new CANcoder(cancoderId, canbus);

        configureDriveMotor(config);
        configureSteerMotor(config);
        configureCancoder(config);

        _drivePosition = _driveMotor.getPosition();
        _driveVelocity = _driveMotor.getVelocity();
        _steerPosition = _steerMotor.getPosition();
        _steerVelocity = _steerMotor.getVelocity();

        double wheelCircumference = 2 * Math.PI * config.wheelRadius();
        _driveRotationsPerMeter = config.driveGearRatio() / wheelCircumference;
        _couplingRatio = config.couplingRatio();

        BaseStatusSignal.setUpdateFrequencyForAll(
                250.0, _drivePosition, _driveVelocity, _steerPosition);

        BaseStatusSignal.setUpdateFrequencyForAll(50, _steerVelocity);

        _driveMotor.optimizeBusUtilization();
        _steerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(_drivePosition, _driveVelocity, _steerPosition, _steerVelocity);

        double driveRot =
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(_drivePosition, _driveVelocity);

        double steerRot =
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(_steerPosition, _steerVelocity);

        driveRot -= steerRot * _couplingRatio;

        inputs.drivePositionMeters = driveRot / _driveRotationsPerMeter;
        inputs.driveVelocityMPS = _driveVelocity.getValueAsDouble() / _driveRotationsPerMeter;
        inputs.driveAppliedVolts = _driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveCurrentAmps = _driveMotor.getStatorCurrent().getValueAsDouble();
        inputs.driveTempCelsius = _driveMotor.getDeviceTemp().getValueAsDouble();

        inputs.steerAngle = Rotation2d.fromRotations(steerRot);
        inputs.steerVelocityRadPerSec = _steerVelocity.getValueAsDouble() * 2.0 * Math.PI;
        inputs.steerAppliedVolts = _steerMotor.getMotorVoltage().refresh().getValueAsDouble();
        inputs.steerCurrentAmps = _steerMotor.getStatorCurrent().getValueAsDouble();
        inputs.steerTempCelsius = _steerMotor.getDeviceTemp().getValueAsDouble();

        inputs.absoluteAngle =
                Rotation2d.fromRotations(_cancoder.getAbsolutePosition().getValueAsDouble());

        inputs.faults =
                _driveMotor.getFaultField().getValue() | (_steerMotor.getFaultField().getValue() << 16);
    }

    @Override
    public void writeOutputs(SwerveModuleOutputs outputs) {
        switch (outputs.driveControlMode) {
            case VOLTAGE:
                _driveMotor.setControl(_driveVoltageReq.withOutput(outputs.driveVoltage));
                break;
            case VELOCITY:
                double velocitySetpoint = outputs.driveVelocitySetpointMPS * _driveRotationsPerMeter;
                _driveMotor.setControl(_driveVelocityReq.withVelocity(velocitySetpoint));
                // .withFeedForward(outputs.driveFeedforwardVolts));
                break;
            default:
                _driveMotor.setControl(new VoltageOut(0));
                break;
        }

        switch (outputs.steerControlMode) {
            case POSITION:
                _steerMotor.setControl(
                        _steerMotionMagicReq.withPosition(outputs.steerAngleSetpoint.getRotations()));
                // .withFeedForward(outputs.steerFeedforwardVolts));
                break;
            case VOLTAGE:
                _steerMotor.setControl(new VoltageOut(outputs.steerVoltage));
                break;
            default:
                _steerMotor.setControl(new VoltageOut(0));
                break;
        }
    }

    @Override
    public void reset() {
        _steerMotor.setPosition(_cancoder.getAbsolutePosition().getValue());
    }

    private void configureDriveMotor(SwerveModuleConfig config) {
        var driveConfigs = new TalonFXConfiguration();
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfigs.MotorOutput.Inverted =
                config.driveInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        driveConfigs.Slot0.kP = config.drivePID().kP();
        driveConfigs.Slot0.kI = config.drivePID().kI();
        driveConfigs.Slot0.kD = config.drivePID().kD();
        driveConfigs.Slot0.kS = config.driveKs();
        driveConfigs.Slot0.kV = config.driveKv();
        driveConfigs.Slot0.kA = config.driveKa();

        driveConfigs.TorqueCurrent.TorqueNeutralDeadband = 0.7;
        driveConfigs.CurrentLimits.StatorCurrentLimit = config.driveCurrentLimit();
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = config.driveCurrentLimit();
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -config.driveCurrentLimit();

        driveConfigs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        _driveMotor.getConfigurator().apply(driveConfigs);
        // CTREUtil.applyConfiguration(_driveMotor, driveConfigs);
    }

    private void configureSteerMotor(SwerveModuleConfig config) {
        var steerConfigs = new TalonFXConfiguration();
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfigs.MotorOutput.Inverted =
                config.steerInverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        steerConfigs.Slot0.kP = config.steerPID().kP();
        steerConfigs.Slot0.kI = config.steerPID().kI();
        steerConfigs.Slot0.kD = config.steerPID().kD();
        steerConfigs.Slot0.kS = config.steerKs();
        steerConfigs.Slot0.kV = config.steerKv();
        steerConfigs.Slot0.kA = config.steerKa();

        steerConfigs.CurrentLimits.StatorCurrentLimit = config.steerCurrentLimit();
        steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        steerConfigs.MotionMagic.MotionMagicCruiseVelocity = config.steerMotionCruiseVelocity();
        steerConfigs.MotionMagic.MotionMagicAcceleration = config.steerMotionAcceleration();

        steerConfigs.Feedback.FeedbackRemoteSensorID = _cancoder.getDeviceID();
        steerConfigs.Feedback.RotorToSensorRatio = config.steerGearRatio();
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

        steerConfigs.TorqueCurrent.PeakForwardTorqueCurrent = config.steerCurrentLimit();
        steerConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -config.steerCurrentLimit();
        // steerConfigs.CurrentLimits.SupplyCurrentLimit = config.steerCurrentLimit();

        _steerMotor.getConfigurator().apply(steerConfigs);
        // CTREUtil.applyConfiguration(_steerMotor, steerConfigs);
    }

    private void configureCancoder(SwerveModuleConfig config) {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        _cancoderConfigs.MagnetSensor.MagnetOffset = config.absoluteEncoderOffset();
        _cancoder.getConfigurator().apply(_cancoderConfigs);
        // CTREUtil.applyConfiguration(_cancoder, _cancoderConfigs);
    }

    @Override
    public void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {
        Slot0Configs config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kV;
        config.kA = kA;
        config.kS = kS;
        config.kG = kG;

        CTREUtil.applyConfiguration(_driveMotor, config);
    }
}
