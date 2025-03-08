package org.frc5687.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.StrobeAnimation;
import java.util.Optional;

public enum LightState {
    OFF(0, 0, 0),
    SOLID_GREEN(6, 62, 57),
    FLASHING_GREEN(new StrobeAnimation(6,62, 57, 0, 0.05, -1)),
    SOLID_WHITE(100, 100, 100),
    FLASHING_WHITE(new StrobeAnimation(100, 100, 100, 0, 0.05, -1)),
    BLUE(0, 0, 255),
    RED(255, 0, 0),
    PURPLE(255, 0, 255);

    public final Optional<Animation> _animation;
    public final int r;
    public final int g;
    public final int b;

    LightState(int r, int g, int b) {
        _animation = Optional.empty();
        this.r = r;
        this.g = g;
        this.b = b;
    }

    LightState(Animation animation) {
        _animation = Optional.of(animation);
        // colors are irrelevant
        this.r = 0;
        this.g = 0;
        this.b = 0;
    }
}
