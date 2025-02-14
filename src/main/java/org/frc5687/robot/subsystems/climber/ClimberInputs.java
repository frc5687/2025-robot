package org.frc5687.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseInputs;

public class ClimberInputs extends BaseInputs {
    @Logged(name = "Climber Motor Angle (rads)", importance = Logged.Importance.CRITICAL)
    public double motorAngleRads = 0.0;

    @Logged(name = "Climber Supply Current (A)", importance = Logged.Importance.CRITICAL)
    public double supplyCurrent = 0.0;

    @Logged(name = "Climber Stator Current (A)", importance = Logged.Importance.CRITICAL)
    public double statorCurrent = 0.0;
}
