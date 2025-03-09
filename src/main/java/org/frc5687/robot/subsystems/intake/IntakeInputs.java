package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class IntakeInputs extends BaseInputs {
    @Logged(name = "Arm Angle(Rads)", importance = Logged.Importance.CRITICAL)
    public double armAngleRads = 0;

    @Logged(name = "Is Coral Detected", importance = Logged.Importance.CRITICAL)
    public boolean isCoralDetected = false;
}
