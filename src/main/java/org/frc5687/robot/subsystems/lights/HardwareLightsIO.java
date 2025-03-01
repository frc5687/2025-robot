package org.frc5687.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import org.frc5687.robot.RobotMap;

public class HardwareLightsIO implements LightsIO {
    private CANdle _candle;
    private LightState _prevLightState;

    public HardwareLightsIO() {
        _candle = new CANdle(RobotMap.CAN.CANDLE);
        _prevLightState = null;
    }

    @Override
    public void updateInputs(LightInputs inputs) {}

    @Override
    public void writeOutputs(LightOutputs outputs) {
        LightState newState = outputs.desiredState;
        if (_prevLightState != null && newState == _prevLightState)
            return; // we have already set the state to this,, no need to continue.

        if (newState._animation.isPresent()) {
            _candle.animate(newState._animation.get());
        } else {
            _candle.clearAnimation(0);
            _candle.setLEDs(newState.r, newState.g, newState.b);
        }

        _prevLightState = newState;
    }
}
