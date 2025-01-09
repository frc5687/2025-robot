package org.frc5687.robot.subsystems.intake;

import javax.tools.DocumentationTool.Location;

import org.frc5687.robot.util.BaseInputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public class IntakeInputs extends BaseInputs {
    @Logged(name = "Roller Velocity(RadsPS)", importance = Logged.Importance.CRITICAL)
    public double rollerVelocityRadperSec = 0;

    @Logged(name = "Roller Temperature C", importance = Logged.Importance.DEBUG)
    public double rollerTemperatureCelsius = 0;

    @Logged(name = "Intake Velocity(RadsPS)", importance = Logged.Importance.CRITICAL)
    public double intakeVelocityRadperSec = 0;

    @Logged(name = "Intake Temperature C", importance = Logged.Importance.DEBUG)
    public double intakeTemperatureCelsius = 0;
}
