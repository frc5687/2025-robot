package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class IntakeOutputs extends BaseOutputs {
    @Logged(name = "Roller Voltage Command (V)", importance = Logged.Importance.CRITICAL)
    public double rollerVoltage = 0.0;

    @Logged(name = "Intake Voltage Command (V)", importance = Logged.Importance.CRITICAL)
    public double intakeVoltage = 0.0;

    @Logged(name = "Desired Angle (rad)", importance = Logged.Importance.CRITICAL)
    public double desiredAngleRad = 0.0;

    @Logged(name = "Dynamics Feed Forward (V)", importance = Logged.Importance.DEBUG)
    public double dynamicsFF = 0.0;

    @Logged(name = "Intake Disabled", importance = Logged.Importance.CRITICAL)
    public boolean disabled = false;
}
