package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class ElevatorInputs extends BaseInputs {
    @Logged(name = "Elevator Height Position (m)", importance = Logged.Importance.CRITICAL)
    public double heightPositionMeters = ElevatorState.STOWED.getHeight();

    @Logged(name = "First Stage Velocity (mps)", importance = Logged.Importance.CRITICAL)
    public double firstStageVelocityMPS = 0.0;

    @Logged(name = "Platform Motors Current (A)", importance = Logged.Importance.CRITICAL)
    public double[] platformMotorCurrents = new double[2];

    @Logged(name = "Platform Acceleration (m/s^2)", importance = Logged.Importance.CRITICAL)
    public double platformAcceleration = 0.0;

    @Logged(name = "Stage 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d stagePose = new Pose3d();

    @Logged(name = "Platform 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d platformPose = new Pose3d();

    @Logged(name = "Laser Measurement (m)", importance = Logged.Importance.CRITICAL)
    public double laserSensorElevatorHeightMeters = -1.0;
}
