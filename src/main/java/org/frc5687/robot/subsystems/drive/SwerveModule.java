package org.frc5687.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.frc5687.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final ModuleIO _io;
    private final ModuleIO.ModuleIOInputs _inputs = new ModuleIO.ModuleIOInputs();
    private final Translation2d _modulePosition;
    private final String _moduleName;
    private SwerveModuleState _goalState = new SwerveModuleState();

    public static class ModuleConfiguration {
        public String moduleName = "";
        public Translation2d position = new Translation2d();
        public double encoderOffset = 0.0;
        public String canBus = "rio";
    }

    public SwerveModule(ModuleConfiguration config, int driveID, int turnID, int encoderID) {
        _io = createIO(driveID, turnID, encoderID, config.canBus, config.encoderOffset);
        _modulePosition = config.position;
        _moduleName = config.moduleName;
    }

    protected ModuleIO createIO(int driveID, int turnID, int encoderID, String canBus, double offset) {
        return new ModuleIOCTRE(driveID, turnID, encoderID, canBus, offset);
    }
    
    public void periodic() {
        _io.updateInputs(_inputs);
    }
    
    public void setGoalState(SwerveModuleState state) {
        // optimize goal state
        Rotation2d currentAngle = _inputs.turnAbsolutePosition;
        state.optimize(currentAngle);
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
        state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
        _goalState = state;

        // send motor setpoints
        _io.runDriveVelocitySetpoint(
            state.speedMetersPerSecond * Constants.SwerveModule.GEAR_RATIO_DRIVE / (2 * Math.PI * Constants.SwerveModule.WHEEL_RADIUS), 
            0.0
        );
        _io.runTurnPositionSetpoint(state.angle.getRadians());
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            _inputs.driveVelocityRadsPerSec * Constants.SwerveModule.WHEEL_RADIUS,
            _inputs.turnAbsolutePosition);
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            _inputs.drivePositionRads * Constants.SwerveModule.WHEEL_RADIUS,
            _inputs.turnAbsolutePosition);
    }
    
    public Translation2d getModuleLocation() {
        return _modulePosition;
    }

    public void updateDashboard() {
        Logger.recordOutput(_moduleName + "/goalSteerAngleRadians", _goalState.angle.getRadians());
        Logger.recordOutput(_moduleName + "/steerAngleRadians", _inputs.turnAbsolutePosition.getRadians());
        Logger.recordOutput(_moduleName + "/goalSpeedMetersPerSecond", _goalState.speedMetersPerSecond);
        Logger.recordOutput(_moduleName + "/speedMetersPerSecond", getState().speedMetersPerSecond);
    }
}