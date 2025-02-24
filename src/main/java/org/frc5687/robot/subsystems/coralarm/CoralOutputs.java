package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

public class CoralOutputs extends BaseOutputs {
    @Logged(name = "Desired Angle (rad)", importance = Logged.Importance.CRITICAL)
    public double desiredAngleRad = CoralState.IDLE.getArmAngle();

    @Logged(name = "Controller Output", importance = Logged.Importance.DEBUG)
    public double controllerOutput = 0.0;

    @Logged(name = "Desired Voltage (V)", importance = Logged.Importance.DEBUG)
    public double voltageCommand = 0.0;

    @Logged(name = "Dynamics Feed Forward (V)", importance = Logged.Importance.DEBUG)
    public double dynamicsFF = 0.0;

    @Logged(
            name = "Wheel Control Type (is position controlled?)",
            importance = Logged.Importance.DEBUG)
    public boolean wheelPositionControl = false;

    @Logged(name = "Wheel Voltage Command", importance = Logged.Importance.DEBUG)
    public double wheelVoltageCommand = 0.0;

    @Logged(name = "Wheel Position Command", importance = Logged.Importance.DEBUG)
    public double wheelPositionCommand = 0.0;
}
