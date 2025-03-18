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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import org.frc5687.robot.util.CTREUtil;

public class CTRESwerveModuleIO implements SwerveModuleIO {
    private final TalonFX _driveMotor;
    private final TalonFX _steerMotor;
    private final CANcoder _cancoder;

    private final StatusSignal<Angle> _drivePosition;
    private final StatusSignal<AngularVelocity> _driveVelocity;
    private final StatusSignal<Angle> _steerPosition;
    private final StatusSignal<AngularVelocity> _steerVelocity;

    private final StatusSignal<Voltage> _driveMotorVoltage;
    private final StatusSignal<Current> _driveStatorCurrent;
    private final StatusSignal<Temperature> _driveTemp;
    private final StatusSignal<Voltage> _steerMotorVoltage;
    private final StatusSignal<Current> _steerStatorCurrent;
    private final StatusSignal<Temperature> _steerTemp;
    private final StatusSignal<Angle> _cancoderAbsolutePosition;

    private final PositionTorqueCurrentFOC _steerMotionMagicReq =
            new PositionTorqueCurrentFOC(0).withUpdateFreqHz(1000.0).withSlot(0);
    private final VelocityTorqueCurrentFOC _driveVelocityReq =
            new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(1000.0);
    private final VoltageOut _driveVoltageReq = new VoltageOut(0);

    private final double _driveRotationsPerMeter;
    private final double _couplingRatio;

    private final List<BaseStatusSignal> _allStatusSignals = new ArrayList<>();

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
        _driveMotorVoltage = _driveMotor.getMotorVoltage();
        _driveStatorCurrent = _driveMotor.getStatorCurrent();
        _driveTemp = _driveMotor.getDeviceTemp();
        _steerMotorVoltage = _steerMotor.getMotorVoltage();
        _steerStatorCurrent = _steerMotor.getStatorCurrent();
        _steerTemp = _steerMotor.getDeviceTemp();
        _cancoderAbsolutePosition = _cancoder.getAbsolutePosition();

        double wheelCircumference = 2 * Math.PI * config.wheelRadius();
        _driveRotationsPerMeter = config.driveGearRatio() / wheelCircumference;
        _couplingRatio = config.couplingRatio();

        _allStatusSignals.add(_drivePosition);
        _allStatusSignals.add(_driveVelocity);
        _allStatusSignals.add(_steerPosition);
        _allStatusSignals.add(_steerVelocity);
        _allStatusSignals.add(_driveMotorVoltage);
        _allStatusSignals.add(_driveStatorCurrent);
        _allStatusSignals.add(_driveTemp);
        _allStatusSignals.add(_steerMotorVoltage);
        _allStatusSignals.add(_steerStatorCurrent);
        _allStatusSignals.add(_steerTemp);
        _allStatusSignals.add(_cancoderAbsolutePosition);

        BaseStatusSignal.setUpdateFrequencyForAll(
                250.0, _drivePosition, _driveVelocity, _steerPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, _steerVelocity, _cancoderAbsolutePosition, _driveMotorVoltage, _steerMotorVoltage);
        BaseStatusSignal.setUpdateFrequencyForAll(
                10.0, _driveStatorCurrent, _steerStatorCurrent, _driveTemp, _steerTemp);

        _driveMotor.optimizeBusUtilization();
        _steerMotor.optimizeBusUtilization();
        _cancoder.optimizeBusUtilization();
    }

    public List<BaseStatusSignal> getStatusSignals() {
        return _allStatusSignals;
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        BaseStatusSignal.refreshAll(_allStatusSignals.toArray(new BaseStatusSignal[0]));

        double driveRot =
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(_drivePosition, _driveVelocity);

        double steerRot =
                BaseStatusSignal.getLatencyCompensatedValueAsDouble(_steerPosition, _steerVelocity);

        driveRot -= steerRot * _couplingRatio;

        inputs.drivePositionMeters = driveRot / _driveRotationsPerMeter;
        inputs.driveVelocityMPS = _driveVelocity.getValueAsDouble() / _driveRotationsPerMeter;
        inputs.driveAppliedVolts = _driveMotorVoltage.getValueAsDouble();
        inputs.driveCurrentAmps = _driveStatorCurrent.getValueAsDouble();
        inputs.driveTempCelsius = _driveTemp.getValueAsDouble();

        inputs.steerAngle = Rotation2d.fromRotations(steerRot);
        inputs.steerVelocityRadPerSec = _steerVelocity.getValueAsDouble() * 2.0 * Math.PI;
        inputs.steerAppliedVolts = _steerMotorVoltage.getValueAsDouble();
        inputs.steerCurrentAmps = _steerStatorCurrent.getValueAsDouble();
        inputs.steerTempCelsius = _steerTemp.getValueAsDouble();

        inputs.absoluteAngle = Rotation2d.fromRotations(_cancoderAbsolutePosition.getValueAsDouble());
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
                break;
            default:
                _driveMotor.setControl(new VoltageOut(0));
                break;
        }

        switch (outputs.steerControlMode) {
            case POSITION:
                _steerMotor.setControl(
                        _steerMotionMagicReq.withPosition(outputs.steerAngleSetpoint.getRotations()));
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
        _steerMotor.setPosition(_cancoderAbsolutePosition.getValue());
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

        _steerMotor.getConfigurator().apply(steerConfigs);
    }

    private void configureCancoder(SwerveModuleConfig config) {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        _cancoderConfigs.MagnetSensor.MagnetOffset = config.absoluteEncoderOffset();
        _cancoder.getConfigurator().apply(_cancoderConfigs);
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

    /**
     * Run characterization on the drive motor
     *
     * @param voltage Voltage to apply
     */
    public void runCharacterization(double voltage) {
        _driveMotor.setControl(new VoltageOut(voltage));
    }
}
