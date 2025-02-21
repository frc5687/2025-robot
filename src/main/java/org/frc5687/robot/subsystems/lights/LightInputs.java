package org.frc5687.robot.subsystems.lights;

import org.frc5687.robot.util.BaseInputs;

import edu.wpi.first.epilogue.Logged;

@Logged
public class LightInputs extends BaseInputs {
    @Logged(name = "Light State", importance = Logged.Importance.CRITICAL)
    public LightState currentState = LightState.PLACEHOLDER;

  }
