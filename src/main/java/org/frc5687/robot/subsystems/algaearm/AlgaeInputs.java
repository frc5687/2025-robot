package org.frc5687.robot.subsystems.algaearm;

import org.frc5687.robot.util.BaseInputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public class AlgaeInputs extends BaseInputs{
    @Logged(name = "Algae Encoder Angle (rads)", importance = Logged.Importance.CRITICAL)
    double algaeEncoderAngle = 0.0;
}
