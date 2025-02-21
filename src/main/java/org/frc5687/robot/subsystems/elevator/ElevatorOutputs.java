package org.frc5687.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class ElevatorOutputs extends BaseOutputs {
    @Logged(name = "Desired Elevator State", importance = Logged.Importance.CRITICAL)
    public ElevatorState desiredState = ElevatorState.STOWED;

    @Logged(name = "Control Mode", importance = Logged.Importance.DEBUG)
    public ElevatorControlMode controlMode = ElevatorControlMode.POSITION;

    @Logged(name = "Desired Height (m)", importance = Logged.Importance.CRITICAL)
    public double desiredHeight = 0.0;

    @Logged(name = "Desired Platform Height World(m)", importance = Logged.Importance.CRITICAL)
    public double desiredPlatformHeightWorldMeters = 0.0;

    @Logged(name = "Platform East Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double voltageCommandEast = 0.0;

    @Logged(name = "Platform West Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double voltageCommandWest = 0.0;

    @Logged(name = "Voltage Feedforward", importance = Logged.Importance.DEBUG)
    public double voltageFeedForward = 0.0;
}
