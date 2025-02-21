package org.frc5687.robot.subsystems.lights;

import edu.wpi.first.epilogue.Logged;

@Logged
public class LightOutputs {
    @Logged(name = "Light Desired State", importance = Logged.Importance.CRITICAL)
    public LightState desiredState = LightState.PLACEHOLDER;
}


