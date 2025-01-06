package org.frc5687.robot.subsystems.drive.modules;

import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule extends OutliersSubsystem<SwerveModuleInputs, SwerveModuleOutputs> {
    private final SwerveModuleConfig _config;
    private SwerveModuleState _desiredState = new SwerveModuleState();
    private boolean _hasReset = false;
    private boolean _isCharacterization = false;

    public SwerveModule(
        SwerveModuleConfig config,
        SubsystemIO<SwerveModuleInputs, SwerveModuleOutputs> io
    ) {
        super(
            io,
            new SwerveModuleInputs(),
            new SwerveModuleOutputs()
        );
        _config = config;
        _inputs.setLogRootName(config.moduleName());
        _outputs.setLogRootName(config.moduleName());
    }

    @Override
    protected void processInputs() {
        if (_inputs.faults != 0) {
            _outputs.driveControlMode = ModuleControlMode.VOLTAGE;
            _outputs.steerControlMode = ModuleControlMode.VOLTAGE;
            _outputs.driveVoltage = 0.0;
            _outputs.steerVoltage = 0.0;
            _hasReset = false;
            return;
        }

        if (!_hasReset && Math.abs(_inputs.steerAngle.minus(_inputs.absoluteAngle).getRadians()) > 0.1) {
            _outputs.steerControlMode = ModuleControlMode.VOLTAGE;
            _outputs.steerVoltage = 0.0;
            _hasReset = true;
            return;
        }
    }

    @Override
    protected void periodic(SwerveModuleInputs inputs, SwerveModuleOutputs outputs) {
        _desiredState.optimize(inputs.steerAngle);

        if (_isCharacterization) {
            outputs.driveControlMode = ModuleControlMode.VOLTAGE;
        } else {
            outputs.driveControlMode = ModuleControlMode.VELOCITY;
            outputs.driveVelocitySetpointMPS = _desiredState.speedMetersPerSecond;
            outputs.driveFeedforwardVolts = _config.driveKs() * Math.signum(_desiredState.speedMetersPerSecond) +
                    _config.driveKv() * _desiredState.speedMetersPerSecond;
        }

        outputs.steerControlMode = ModuleControlMode.POSITION;
        outputs.steerAngleSetpoint = _desiredState.angle;
        outputs.steerFeedforwardVolts = 
            _config.steerKs() * Math.signum(outputs.steerVoltage) +
            _config.steerKv() * inputs.steerVelocityRadPerSec;
    }

    public void setDesiredState(SwerveModuleState state) {
        _desiredState = state;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            _inputs.driveVelocityMPS,
            _inputs.steerAngle
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            _inputs.drivePositionMeters,
            _inputs.steerAngle
        );
    }

    public void runCharacterization(double output) {
        _isCharacterization = true;
        _outputs.driveVoltage = output;
    }

    public void stopCharacterization() {
        _isCharacterization = false;
        _outputs.driveVoltage = 0.0;
    }
}
