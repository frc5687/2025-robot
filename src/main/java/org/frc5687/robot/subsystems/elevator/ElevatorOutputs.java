package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.util.BaseOutputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ElevatorOutputs extends BaseOutputs {
    
    @Logged(name = "Elevator Voltage Command (V)", importance = Logged.Importance.CRITICAL)
    public double elevatorVoltage = 0.0;

    @Logged(name = "Set Position Command (V)", importance = Logged.Importance.CRITICAL)
    public double desiredElevatorPositionMeters = 0.0;

}
