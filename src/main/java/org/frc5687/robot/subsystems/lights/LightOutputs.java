package org.frc5687.robot.subsystems.lights;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

@Logged
public class LightOutputs extends BaseOutputs {
    @Logged(name = "Light Desired State", importance = Logged.Importance.CRITICAL)
    public LightState desiredState = LightState.OFF;
}
