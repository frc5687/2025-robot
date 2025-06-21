package org.frc5687.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseInputs;
import org.littletonrobotics.junction.LogTable.LoggableType;

public class ClimberInputs extends BaseInputs {
    @Logged(name = "Winch Motor Angle (rads)", importance = Logged.Importance.CRITICAL)
    public double motorAngleRads = 0.0;

    @Logged(name = "Winch Supply Current (A)", importance = Logged.Importance.DEBUG)
    public double supplyCurrent = 0.0;

    @Logged(name = "Winch Stator Current (A)", importance = Logged.Importance.DEBUG)
    public double statorCurrent = 0.0;

    @Logged(name = "Climber Supply Current (A)", importance = Logged.Importance.CRITICAL)
    public double climberSupplyCurrent = 0.0;

    @Logged(name = "Climber Stator Current (A)", importance =  Logged.Importance.CRITICAL)
    public double climberStatorCurrent = 0.0;

    @Logged(name = "Climber Sensor", importance = Logged.Importance.CRITICAL)
    public boolean sensor = false;
}
