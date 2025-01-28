package org.frc5687.robot.subsystems.drive;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class DriveInputs extends BaseInputs {
    @Logged(name = "Gyro/Pitch (rad)", importance = Logged.Importance.DEBUG)
    public Rotation2d pitchPosition = new Rotation2d();

    @Logged(name = "Gyro/Roll (rad)", importance = Logged.Importance.DEBUG)
    public Rotation2d rollPosition = new Rotation2d();

    @Logged(name = "Gyro/Yaw (rad)", importance = Logged.Importance.DEBUG)
    public Rotation2d yawPosition = new Rotation2d();

    @Logged(name = "Gyro/Pitch Rate (rad/s)", importance = Logged.Importance.DEBUG)
    public double pitchVelocityRadPerSec = 0.0;

    @Logged(name = "Gyro/Roll Rate (rad/s)", importance = Logged.Importance.DEBUG)
    public double rollVelocityRadPerSec = 0.0;

    @Logged(name = "Gyro/Yaw Rate (rad/s)", importance = Logged.Importance.DEBUG)
    public double yawVelocityRadPerSec = 0.0;

    @Logged(name = "Modules/States", importance = Logged.Importance.DEBUG)
    public SwerveModuleState[] measuredStates = new SwerveModuleState[4];

    @Logged(name = "Modules/Positions", importance = Logged.Importance.DEBUG)
    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    @Logged(name = "Odometry Pose", importance = Logged.Importance.DEBUG)
    public Pose2d odometryPose = new Pose2d();

    public DriveInputs() {
        // Initialize arrays with non-null values to avoid nullptr errors
        for (int i = 0; i < 4; i++) {
            measuredStates[i] = new SwerveModuleState();
            modulePositions[i] = new SwerveModulePosition();
        }
    }
}
