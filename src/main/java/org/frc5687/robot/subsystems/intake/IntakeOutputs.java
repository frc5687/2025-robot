package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class IntakeOutputs extends BaseOutputs {
    @Logged(name = "Roller Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double rollerVoltage = 0.0;

    @Logged(name = "Intake Voltage Command (V)", importance = Logged.Importance.DEBUG)
    public double intakeVoltage = 0.0;

    @Logged(name = "Pivot Target Angle", importance = Logged.Importance.DEBUG)
    public double pivotTargetAngle = IntakeState.IDLE.getValue();

    @Logged(name = "Desired Angle (rad)", importance = Logged.Importance.DEBUG)
    double desiredAngleRad = 0.0;

    @Logged(name = "Controller Output", importance = Logged.Importance.DEBUG)
    double controllerOutput = 0.0;
}
