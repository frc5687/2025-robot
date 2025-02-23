package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class ElevatorInputs extends BaseInputs {
    @Logged(name = "Elevator State", importance = Logged.Importance.CRITICAL)
    public ElevatorState elevatorState = ElevatorState.STOWED;

    @Logged(name = "Motor Height (m)", importance = Logged.Importance.CRITICAL)
    public double motorHeightMeters = 0.0;

    @Logged(name = "First Stage Velocity (mps)", importance = Logged.Importance.CRITICAL)
    public double firstStageVelocityMPS = 0.0;

    @Logged(name = "Platform Motors Current (A)", importance = Logged.Importance.CRITICAL)
    public double[] platformMotorCurrents = new double[2];

    @Logged(name = "Stage 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d stagePose = new Pose3d();

    @Logged(name = "Platform 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d platformPose = new Pose3d();

    @Logged(name = "Laser Elevator Height (m)", importance = Logged.Importance.CRITICAL)
    public double laserSensorElevatorHeightMeters = -1.0;
}
