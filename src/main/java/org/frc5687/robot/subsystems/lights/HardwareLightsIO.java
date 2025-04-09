package org.frc5687.robot.subsystems.lights;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.EpilogueLog;

public class HardwareLightsIO implements LightsIO, EpilogueLog {
    private CANdle _candle;
    private LightState _prevLightState;
    private Color _dashboardPrevColor;

    public HardwareLightsIO() {
        _candle = new CANdle(RobotMap.CAN.CANDLE);
        _prevLightState = null;
        _dashboardPrevColor = new Color();
    }

    @Override
    public void updateInputs(LightInputs inputs) {}

    @Override
    public void writeOutputs(LightOutputs outputs) {
        Color newColor;
        if (outputs.desiredState == LightState.SOLID_WHITE) {
            // hardcode white because we want to actually send #fff and not the dimmer led values
            newColor = new Color("#ffffff");
        } else if (outputs.desiredState._animation.isEmpty()) {
            // scuffed lol -- this means it's a solid color
            newColor = new Color(outputs.desiredState.r, outputs.desiredState.g, outputs.desiredState.b);
        } else if (outputs.desiredState == LightState.FLASHING_GREEN) {
            var activeColor = new Color("#00ff00");
            if (blink()) {
                newColor = activeColor;
            } else {
                newColor = new Color();
            }
        } else { // flashing white
            var activeColor = new Color("#ffffff");
            if (blink()) {
                newColor = activeColor;
            } else {
                newColor = new Color();
            }
        }

        if (newColor != _dashboardPrevColor) {
            log("LIGHTS HEXSTRING", newColor.toHexString(), Importance.CRITICAL);
            _dashboardPrevColor = newColor;
        }

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

    private boolean blink() {
        return Timer.getFPGATimestamp() % 0.24 > 0.12;
    }

    @Override
    public String getLogBase() {
        return "DashboardLights";
    }
}
