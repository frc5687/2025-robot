package org.frc5687.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import org.frc5687.robot.RobotMap;

public class HardwareLightsIO implements LightsIO {
    private CANdle _candle;

    public HardwareLightsIO() {
        _candle = new CANdle(RobotMap.CAN.CANDLE);
        CANdleConfiguration configAll = new CANdleConfiguration();
    }

    @Override
    public void updateInputs(LightInputs inputs) {}

    @Override
    public void writeOutputs(LightOutputs outputs) {
        _candle.animate(outputs.desiredState.getAnimation());
    }
}
