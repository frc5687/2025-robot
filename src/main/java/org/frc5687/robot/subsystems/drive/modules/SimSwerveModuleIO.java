package org.frc5687.robot.subsystems.drive.modules;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimSwerveModuleIO implements SwerveModuleIO {
    // Simulation constants
    private static final double SIM_DRIVE_KP = 0.05;
    private static final double SIM_DRIVE_KD = 0.0;
    private static final double SIM_DRIVE_KS = 0.0;
    private static final double SIM_DRIVE_KV = 0.91035; // volt * sec/rotation
    private static final double SIM_TURN_KP = 8.0;
    private static final double SIM_TURN_KD = 0.0;

    private final TalonFX _driveMotor;
    private final TalonFX _steerMotor;
    private final CANcoder _cancoder;

    private final TalonFXSimState _driveSim;
    private final TalonFXSimState _steerSim;
    private final CANcoderSimState _cancoderSim;

    private final DCMotorSim _driveMotorSim;
    private final DCMotorSim _steerMotorSim;

    private final PIDController _driveController = new PIDController(SIM_DRIVE_KP, 0, SIM_DRIVE_KD);
    private final PIDController _steerController = new PIDController(SIM_TURN_KP, 0, SIM_TURN_KD);

    private final double _driveRotationsPerMeter;
    private final double _loopPeriodSecs = 0.02; // 50Hz

    private boolean _driveClosedLoop = false;
    private boolean _steerClosedLoop = false;
    private double _driveFeedforwardVolts = 0.0;
    private double _driveAppliedVolts = 0.0;
    private double _steerAppliedVolts = 0.0;

    public SimSwerveModuleIO(
            SwerveModuleConfig config,
            int driveMotorId,
            int steerMotorId,
            int cancoderId,
            String canbus) {
        _driveMotor = new TalonFX(driveMotorId, canbus);
        _steerMotor = new TalonFX(steerMotorId, canbus);
        _cancoder = new CANcoder(cancoderId, canbus);

        _driveSim = _driveMotor.getSimState();
        _steerSim = _steerMotor.getSimState();
        _cancoderSim = _cancoder.getSimState();

        _driveMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60Foc(1), 0.025, config.driveGearRatio()),
                        DCMotor.getKrakenX60Foc(1));

        _steerMotorSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60Foc(1), 0.004, config.steerGearRatio()),
                        DCMotor.getKrakenX60Foc(1));

        double wheelCircumference = 2 * Math.PI * config.wheelRadius();
        _driveRotationsPerMeter = config.driveGearRatio() / wheelCircumference;

        _steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        double voltage = RobotController.getBatteryVoltage();

        if (_driveClosedLoop) {
            _driveAppliedVolts =
                    _driveFeedforwardVolts
                            + _driveController.calculate(_driveMotorSim.getAngularVelocityRadPerSec());
        }
        if (_steerClosedLoop) {
            _steerAppliedVolts = _steerController.calculate(_steerMotorSim.getAngularPositionRad());
        }

        _driveMotorSim.setInputVoltage(MathUtil.clamp(_driveAppliedVolts, -12.0, 12.0));
        _steerMotorSim.setInputVoltage(MathUtil.clamp(_steerAppliedVolts, -12.0, 12.0));
        _driveMotorSim.update(_loopPeriodSecs);
        _steerMotorSim.update(_loopPeriodSecs);

        inputs.drivePositionMeters =
                _driveMotorSim.getAngularPositionRad() / (2 * Math.PI * _driveRotationsPerMeter);
        inputs.driveVelocityMPS =
                _driveMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI * _driveRotationsPerMeter);
        inputs.driveAppliedVolts = _driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(_driveMotorSim.getCurrentDrawAmps());
        inputs.driveTempCelsius = 25.0;

        inputs.steerAngle = new Rotation2d(_steerMotorSim.getAngularPositionRad());
        inputs.steerVelocityRadPerSec = _steerMotorSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = _steerAppliedVolts;
        inputs.steerCurrentAmps = Math.abs(_steerMotorSim.getCurrentDrawAmps());
        inputs.steerTempCelsius = 25.0;

        _driveSim.setRawRotorPosition(_driveMotorSim.getAngularPositionRad() / (2 * Math.PI));
        _driveSim.setRotorVelocity(_driveMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI));
        _steerSim.setRawRotorPosition(_steerMotorSim.getAngularPositionRad() / (2 * Math.PI));
        _steerSim.setRotorVelocity(_steerMotorSim.getAngularVelocityRadPerSec() / (2 * Math.PI));

        inputs.absoluteAngle = inputs.steerAngle;
        inputs.faults = 0;
    }

    @Override
    public void writeOutputs(SwerveModuleOutputs outputs) {
        switch (outputs.driveControlMode) {
            case VOLTAGE:
                _driveClosedLoop = false;
                _driveAppliedVolts = outputs.driveVoltage;
                break;
            case VELOCITY:
                _driveClosedLoop = true;
                _driveFeedforwardVolts =
                        SIM_DRIVE_KS * Math.signum(outputs.driveVelocitySetpointMPS)
                                + SIM_DRIVE_KV * outputs.driveVelocitySetpointMPS
                                + outputs.driveFeedforwardVolts;
                _driveController.setSetpoint(
                        outputs.driveVelocitySetpointMPS * 2 * Math.PI * _driveRotationsPerMeter);
                break;
            default:
                _driveClosedLoop = false;
                _driveAppliedVolts = 0.0;
                break;
        }

        switch (outputs.steerControlMode) {
            case POSITION:
                _steerClosedLoop = true;
                _steerController.setSetpoint(outputs.steerAngleSetpoint.getRadians());
                break;
            case VOLTAGE:
                _steerClosedLoop = false;
                _steerAppliedVolts = outputs.steerVoltage;
                break;
            default:
                _steerClosedLoop = false;
                _steerAppliedVolts = 0.0;
                break;
        }
    }

    @Override
    public void reset() {
        _driveMotorSim.setState(0.0, 0.0);
        _steerMotorSim.setState(0.0, 0.0);
        _driveController.reset();
        _steerController.reset();
        _driveSim.setRawRotorPosition(0);
        _steerSim.setRawRotorPosition(0);
        _cancoderSim.setRawPosition(0);
    }
}
