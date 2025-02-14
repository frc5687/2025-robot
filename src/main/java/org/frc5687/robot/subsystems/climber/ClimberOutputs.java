package org.frc5687.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

public class ClimberOutputs extends BaseOutputs {
    @Logged(name = "Climber Motor Setpoint (rads)", importance = Logged.Importance.CRITICAL)
    public double motorSetpointRads = 0.0;
}
