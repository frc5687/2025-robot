package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class AlgaeInputs extends BaseInputs {
    @Logged(name = "Algae Encoder Angle (rads)", importance = Logged.Importance.CRITICAL)
    double algaeEncoderAngle = 0.0;
}
