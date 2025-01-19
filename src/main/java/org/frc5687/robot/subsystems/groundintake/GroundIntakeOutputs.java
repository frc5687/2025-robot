package org.frc5687.robot.subsystems.groundintake;

import org.frc5687.robot.util.BaseOutputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public class GroundIntakeOutputs extends BaseOutputs{
    @Logged(name = "Pivot Target Angle(rads)", importance = Logged.Importance.CRITICAL)
    public double pivotTargetAngle = 0.0;

    @Logged(name = "Right Roller Voltage Command", importance = Logged.Importance.CRITICAL)
    public double rightRollerVoltage = 0.0;

    @Logged(name = "Left Roller Voltage Command", importance = Logged.Importance.CRITICAL)
    public double leftRollerVoltage = 0.0;
}
