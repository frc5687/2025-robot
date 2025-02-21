package org.frc5687.robot.subsystems.lights;

public enum LightState {
    PLACEHOLDER(0);

    private final double _brightness;

    LightState(double brightness) {
        _brightness = brightness;
    }

    public double getBrightness() {
        return _brightness;
    }

}
