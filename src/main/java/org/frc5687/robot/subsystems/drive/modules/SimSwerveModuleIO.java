package org.frc5687.robot.subsystems.drive.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimSwerveModuleIO implements SwerveModuleIO {
    private static final double SIM_DRIVE_KP = 0.25;
    private static final double SIM_DRIVE_KD = 0.0;
    private static final double SIM_STEER_KP = 8.0;
    private static final double SIM_STEER_KD = 0.0;
    private static final double LOOP_PERIOD_SECS = 0.02; // 50Hz

    private final DCMotorSim _driveSim;
    private final DCMotorSim _steerSim;

    private final PIDController _driveController = new PIDController(SIM_DRIVE_KP, 0, SIM_DRIVE_KD);
    private final PIDController _steerController = new PIDController(SIM_STEER_KP, 0, SIM_STEER_KD);

    private final double _driveGearRatio;
    private final double _wheelCircumference;

    private boolean _driveClosedLoop = false;
    private boolean _steerClosedLoop = false;
    private double _driveFeedforwardVolts = 0.0;
    private double _driveAppliedVolts = 0.0;
    private double _steerAppliedVolts = 0.0;

    public SimSwerveModuleIO(SwerveModuleConfig config) {
        _driveGearRatio = config.driveGearRatio();

        _driveSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025, _driveGearRatio),
                        DCMotor.getKrakenX60Foc(1));

        _steerSim =
                new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(
                                DCMotor.getKrakenX60Foc(1), 0.004, config.steerGearRatio()),
                        DCMotor.getKrakenX60Foc(1));

        _wheelCircumference = 2 * Math.PI * config.wheelRadius();
        _steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        double voltage = RobotController.getBatteryVoltage();

        if (_driveClosedLoop) {
            _driveAppliedVolts =
                    _driveFeedforwardVolts
                            + _driveController.calculate(_driveSim.getAngularVelocityRadPerSec());
        }

        if (_steerClosedLoop) {
            _steerAppliedVolts = _steerController.calculate(_steerSim.getAngularPositionRad());
        }

        _driveSim.setInputVoltage(MathUtil.clamp(_driveAppliedVolts, -voltage, voltage));
        _steerSim.setInputVoltage(MathUtil.clamp(_steerAppliedVolts, -voltage, voltage));
        _driveSim.update(LOOP_PERIOD_SECS);
        _steerSim.update(LOOP_PERIOD_SECS);

        double drivePositionRad = _driveSim.getAngularPositionRad();
        inputs.drivePositionMeters = drivePositionRad * _wheelCircumference;

        double driveVelocityRadPerSec = _driveSim.getAngularVelocityRadPerSec();
        inputs.driveVelocityMPS = driveVelocityRadPerSec * _wheelCircumference;

        inputs.driveAppliedVolts = _driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(_driveSim.getCurrentDrawAmps());
        inputs.driveTempCelsius = 25.0;

        inputs.steerAngle = new Rotation2d(_steerSim.getAngularPositionRad());
        inputs.steerVelocityRadPerSec = _steerSim.getAngularVelocityRadPerSec();
        inputs.steerAppliedVolts = _steerAppliedVolts;
        inputs.steerCurrentAmps = Math.abs(_steerSim.getCurrentDrawAmps());
        inputs.steerTempCelsius = 25.0;

        inputs.absoluteAngle = inputs.steerAngle;
        inputs.faults = 0;
        // System.out.println("SimSwerveModuleIO Update time: " + Timer.getTimestamp());
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
                _driveFeedforwardVolts = outputs.driveFeedforwardVolts;
                double targetRadPerSec = outputs.driveVelocitySetpointMPS / _wheelCircumference;
                _driveController.setSetpoint(targetRadPerSec);
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
        _driveSim.setState(0.0, 0.0);
        _steerSim.setState(0.0, 0.0);
        _driveController.reset();
        _steerController.reset();
    }
}
