package org.frc5687.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public SwerveModuleState[] measuredStates = {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        };
        
        public SwerveModulePosition[] measuredPositions = {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
        
        public Rotation2d heading = new Rotation2d();
        public double pitchDegrees = 0.0;
        public double rollDegrees = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DriveIOInputs inputs) {}

    /** Set states for all swerve modules */
    public default void setModuleStates(SwerveModuleState[] states) {}

    /** Zero the gyroscope */
    public default void zeroGyroscope() {}
}