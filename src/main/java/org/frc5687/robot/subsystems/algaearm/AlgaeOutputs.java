package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

public class AlgaeOutputs extends BaseOutputs {
    @Logged(name = "Desired Algae State", importance = Logged.Importance.CRITICAL)
    AlgaeState desiredState = AlgaeState.IDLE;

    @Logged(name = "Desired Angle (rad)", importance = Logged.Importance.CRITICAL)
    double desiredAngleRad = 0.0;

    @Logged(name = "Controller Output", importance = Logged.Importance.DEBUG)
    double controllerOutput = 0.0;

    @Logged(name = "Desired Voltage (V)", importance = Logged.Importance.DEBUG)
    double voltageCommand = 0.0;

    @Logged(name = "Desired Voltage Feed Forward (V)", importance = Logged.Importance.DEBUG)
    double voltageFeedForward = 0.0;

    @Logged(name = "Wheel Voltage Command", importance = Logged.Importance.DEBUG)
    public double wheelVoltageCommand = 0.0;
}
