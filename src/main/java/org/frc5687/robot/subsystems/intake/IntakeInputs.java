package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseInputs;

@Logged
public class IntakeInputs extends BaseInputs {
    @Logged(name = "Roller Velocity(RadsPS)", importance = Logged.Importance.CRITICAL)
    public double rollerVelocityRadperSec = 0;

    @Logged(name = "Roller Temperature C", importance = Logged.Importance.DEBUG)
    public double rollerTemperatureCelsius = 0;

    @Logged(name = "Absolute Encoder Angle(Rads)", importance = Logged.Importance.CRITICAL)
    public double absoluteEncoderAngleRads = 0;

    @Logged(name = "Arm Angle(Rads)", importance = Logged.Importance.CRITICAL)
    public double armAngleRads = 0;

    @Logged(name = "Intake Velocity (RadsPS)", importance = Logged.Importance.CRITICAL)
    public double angularVelocityRadPerSec = 0;

    @Logged(name = "Intake Current Draw (A)", importance = Logged.Importance.DEBUG)
    public double intakeCurrentDraw = 0.0;

    @Logged(name = "Intake Temperature C", importance = Logged.Importance.DEBUG)
    public double intakeTemperatureCelsius = 0;

    @Logged(name = "Is Coral Detected", importance = Logged.Importance.CRITICAL)
    public boolean isCoralDetected = false;
}
