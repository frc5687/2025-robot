package org.frc5687.robot.subsystems.elevator;

import javax.tools.DocumentationTool.Location;

import org.frc5687.robot.util.BaseInputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public class ElevatorInputs extends BaseInputs {
    
    @Logged(name = "Elevator Velocity(RadPS)", importance = Logged.Importance.CRITICAL)
    public double motorVelocityRadPerSec = 0;
    @Logged(name = "Elevator Position(RadPS)", importance = Logged.Importance.CRITICAL)
    public double motorPositionRad = 0;

    @Logged(name = "Elevator Velocity(MetersPS)", importance = Logged.Importance.CRITICAL)
    public double elevatorVelocityMetersPerSec = 0;
    @Logged(name = "Elevator Velocity(MetersPerSec)", importance = Logged.Importance.CRITICAL)
    public double elevatorPositionMeters = 0;

    

}
