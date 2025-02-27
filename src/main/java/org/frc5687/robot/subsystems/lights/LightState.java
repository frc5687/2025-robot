package org.frc5687.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

public enum LightState {
    TWINKLE_DEFAULT(255, 255, 255);

    private final Animation _animation;

    LightState(int r, int g, int b) {
        _animation = new TwinkleAnimation(r, g, b, 0, 1.0, 100, TwinklePercent.Percent42);
    }

    public Animation getAnimation() {
        return _animation;
    }
}
