package org.frc5687.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.FireAnimation;
import java.util.Optional;

public enum LightState {
    OFF(0, 0, 0),
    GREEN(0, 255, 0),
    DARK_GREEN(0, 100, 0),
    WHITE(100, 100, 100),
    FIRE(new FireAnimation());

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
