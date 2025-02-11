package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class ElevatorOutputs extends BaseOutputs {
    @Logged(name = "Desired Elevator State", importance = Logged.Importance.CRITICAL)
    public ElevatorState desiredState = ElevatorState.STOWED;

    @Logged(name = "Desired Stage Height (m)", importance = Logged.Importance.CRITICAL)
    public double desiredStageHeight = 0.0;

    @Logged(name = "Desired North West Stage Height (m)", importance = Logged.Importance.CRITICAL)
    public double northWestStageHeight = 0.0;

    @Logged(name = "Desired North East Stage Height (m)", importance = Logged.Importance.CRITICAL)
    public double northEastStageHeight = 0.0;

    @Logged(name = "Desired South West Stage Height (m)", importance = Logged.Importance.CRITICAL)
    public double southWestStageHeight = 0.0;

    @Logged(name = "Desired North West Stage Velocity (mps)", importance = Logged.Importance.CRITICAL)
    public double northWestStageVel = 0.0;

    @Logged(name = "Desired North East Stage Velocity (mps)", importance = Logged.Importance.CRITICAL)
    public double northEastStageVel = 0.0;

    @Logged(name = "Desired South West Stage Velocty (mps)", importance = Logged.Importance.CRITICAL)
    public double southWestStageVel = 0.0;

    @Logged(name = "Desired Platform Height World(m)", importance = Logged.Importance.CRITICAL)
    public double desiredPlatformHeightWorldMeters = 0.0;

    @Logged(name = "Desired Platform Pitch (rad)", importance = Logged.Importance.CRITICAL)
    public double desiredPlatformPitchRadians = 0.0;

    @Logged(name = "Desired Platform Roll (rad)", importance = Logged.Importance.CRITICAL)
    public double desiredPlatformRollRadians = 0.0;

    @Logged(name = "Platform North West Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double voltageCommandNorthWest = 0.0;

    @Logged(name = "Platform North East Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double voltageCommandNorthEast = 0.0;

    @Logged(name = "Platform South West Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double voltageCommandSouthWest = 0.0;

    @Logged(name = "Using Hold Position PID")
    public boolean usingPositionHolding = false;

    @Logged(name = "Feedforwards", importance = Logged.Importance.DEBUG)
    public double[] voltageFeedForwards = {0.0, 0.0, 0.0};
}
