package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class ElevatorInputs extends BaseInputs {
    @Logged(name = "Elevator Height Position (m)", importance = Logged.Importance.CRITICAL)
    public double heightPositionMeters = ElevatorState.STOWED.getHeight();

    @Logged(name = "First Stage Velocity (mps)", importance = Logged.Importance.DEBUG)
    public double firstStageVelocityMPS = 0.0;

    @Logged(name = "Platform Motors Current (A)", importance = Logged.Importance.CRITICAL)
    public double[] platformMotorCurrents = new double[2];

    @Logged(name = "Platform Acceleration (m/s^2)", importance = Logged.Importance.DEBUG)
    public double platformAcceleration = 0.0;

    @Logged(name = "Stage 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d stagePose = new Pose3d();

    @Logged(name = "Platform 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d platformPose = new Pose3d();

    @Logged(name = "Laser Measurement (m)", importance = Logged.Importance.CRITICAL)
    public double laserSensorElevatorHeightMeters = -1.0;

    // Enhanced logging for motor synchronization
    @Logged(name = "East Motor Position (m)", importance = Logged.Importance.DEBUG)
    public double eastPositionMeters = 0.0;

    @Logged(name = "West Motor Position (m)", importance = Logged.Importance.DEBUG)
    public double westPositionMeters = 0.0;

    @Logged(name = "East-West Position Difference (m)", importance = Logged.Importance.CRITICAL)
    public double eastWestPositionDifference = 0.0;

    @Logged(name = "East Motor Velocity (mps)", importance = Logged.Importance.DEBUG)
    public double eastVelocityMPS = 0.0;

    @Logged(name = "West Motor Velocity (mps)", importance = Logged.Importance.DEBUG)
    public double westVelocityMPS = 0.0;

    @Logged(name = "East-West Velocity Difference (mps)", importance = Logged.Importance.DEBUG)
    public double eastWestVelocityDifference = 0.0;

    // Safety status
    @Logged(name = "Safety Status", importance = Logged.Importance.CRITICAL)
    public String safetyStatus = "OK";

    @Logged(name = "Elevator Disabled", importance = Logged.Importance.CRITICAL)
    public boolean isDisabled = false;

    @Logged(name = "Position Difference Safety Trip (m)", importance = Logged.Importance.DEBUG)
    public double positionDifferenceSafetyThreshold = 1000000; // 3cm default, configurable

    @Logged(name = "Zeroed", importance = Logged.Importance.DEBUG)
    public boolean zeroed = false;
}
