package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class AlgaeOutputs extends BaseOutputs {
    @Logged(name = "Algae Motor Voltage", importance = Logged.Importance.CRITICAL)
    double algaeMotorVoltage = 0.0;

    @Logged(name = "Algae Pivot Voltage", importance = Logged.Importance.CRITICAL)
    double algaePivotVoltage = 0.0;

    @Logged(name = "Algae Arm Target Angle", importance = Logged.Importance.CRITICAL)
    double algaeArmTargetAngle = 0.0;
}
