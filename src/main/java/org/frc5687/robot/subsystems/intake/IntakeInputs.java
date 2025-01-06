package org.frc5687.robot.subsystems.intake;

import javax.tools.DocumentationTool.Location;

import org.frc5687.robot.util.BaseInputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public class IntakeInputs extends BaseInputs {
    @Logged(name = "Motor Velocity(RadsPS)", importance = Logged.Importance.CRITICAL)
    public double motorVelocityRadperSec = 0;

    @Logged(name = "Motor Temperature C", importance = Logged.Importance.DEBUG)
    public double motorTemperatureCelsius = 0;
}
