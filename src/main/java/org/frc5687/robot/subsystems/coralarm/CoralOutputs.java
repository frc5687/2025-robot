package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

public class CoralOutputs extends BaseOutputs {
    @Logged(name = "Desired Coral State", importance = Logged.Importance.CRITICAL)
    public CoralState desiredState = CoralState.STOWED;

    @Logged(name = "Desired Angle (rad)", importance = Logged.Importance.CRITICAL)
    double desiredAngleRad = 0.0;

    @Logged(name = "Controller Output", importance = Logged.Importance.DEBUG)
    double controllerOutput = 0.0;

    @Logged(name = "Desired Voltage (V)", importance = Logged.Importance.DEBUG)
    double voltageCommand = 0.0;

    @Logged(name = "Desired Voltage Feed Forward (V)", importance = Logged.Importance.DEBUG)
    double voltageFeedForward = 0.0;
}
