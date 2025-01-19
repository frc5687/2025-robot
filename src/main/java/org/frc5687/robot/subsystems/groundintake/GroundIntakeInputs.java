package org.frc5687.robot.subsystems.groundintake;

import org.frc5687.robot.util.BaseInputs;

import edu.wpi.first.epilogue.Logged;
@Logged
public class GroundIntakeInputs extends BaseInputs{

    @Logged(name = "Pivot Angle (rads)", importance = Logged.Importance.CRITICAL)
    public double intakePivotAngle = 0;

    @Logged(name = "Right Roller Velocity(RadsPS)", importance =  Logged.Importance.CRITICAL)
    public double rightRollerVelocity = 0;
    
    @Logged(name = "Left Roller Velocity(RadsPS)", importance =  Logged.Importance.CRITICAL)
    public double leftRollerVelocity = 0;

    @Logged(name = "Right Roller Temperature C", importance = Logged.Importance.DEBUG)
    public double rightRollerTemperatureCelsius = 0;

    @Logged(name = "Roller Temperature C", importance = Logged.Importance.DEBUG)
    public double leftRollerTemperatureCelsius = 0;

    @Logged(name = "Pivot Temperature C", importance = Logged.Importance.DEBUG)
    public double pivotTemperatureCelsius = 0;

}
