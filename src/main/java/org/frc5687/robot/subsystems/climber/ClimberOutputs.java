package org.frc5687.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

public class ClimberOutputs extends BaseOutputs {
    @Logged(name = "Climber Motor Setpoint (rads)", importance = Logged.Importance.CRITICAL)
    public double motorSetpointRads = 0.0;

    @Logged(name = "Climber velocity", importance = Logged.Importance.CRITICAL)
    public double motorVelocityRadPerSec = 0.0;

    @Logged(name = "Climber output voltage (V)", importance = Logged.Importance.CRITICAL)
    public double climberVoltage = 0;

    @Logged(name = "Servo Setpoint (0.0-1.0)", importance = Logged.Importance.CRITICAL)
    public double servoSetpoint = 0.6;
}
