package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose3d;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class ElevatorInputs extends BaseInputs {
    @Logged(name = "Elevator State", importance = Logged.Importance.CRITICAL)
    public ElevatorState elevatorState = ElevatorState.STOWED;

    @Logged(name = "First Stage Position (m)", importance = Logged.Importance.CRITICAL)
    public double firstStagePositionMeters = 0.0;

    @Logged(name = "Stage North East Position (m)", importance = Logged.Importance.CRITICAL)
    public double stageNorthEastPositionMeters = 0.0;

    @Logged(name = "Stage North West Position (m)", importance = Logged.Importance.CRITICAL)
    public double stageNorthWestPositionMeters = 0.0;

    @Logged(name = "Stage South West Position (m)", importance = Logged.Importance.CRITICAL)
    public double stageSouthWestPositionMeters = 0.0;

    @Logged(name = "Platform Height (m)", importance = Logged.Importance.CRITICAL)
    public double platformHeightMeters = 0.0;

    @Logged(name = "Platform Velocity (mps)")
    public double platformVelocityMPS = 0.0;

    @Logged(name = "Platform Roll (rad)", importance = Logged.Importance.CRITICAL)
    public double platformRollRadians = 0.0;

    @Logged(name = "Platform Pitch (rad)", importance = Logged.Importance.CRITICAL)
    public double platformPitchRadians = 0.0;

    @Logged(name = "Platform Motors Current (A)", importance = Logged.Importance.CRITICAL)
    public double[] platformMotorCurrents = new double[3];

    @Logged(name = "Stage 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d stagePose = new Pose3d();

    @Logged(name = "Platform 3D Pose", importance = Logged.Importance.DEBUG)
    public Pose3d platformPose = new Pose3d();
}
