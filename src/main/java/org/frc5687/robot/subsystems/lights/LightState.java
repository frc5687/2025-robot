package org.frc5687.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;


public enum LightState {
    TWINKLE_DEFAULT(0, 255, 255, 255),
    FIRE_DEFAULT(1, 255, 255, 255),
    SINGLE_FADE_DEFAULT(2, 255, 255, 255);
    

    private final Animation _animation;
    LightState(int id, int r, int g, int b) {
        //TODO find better settings for the animation
        switch (id) {
            case 0:
                _animation = new TwinkleAnimation(r, g, b, 0, 1.0, 100, TwinklePercent.Percent42);
                break;
            case 1:
                _animation = new FireAnimation(r, g, b, 0.5, 1.0, false, 0);
                break;
            case 2:
                _animation = new SingleFadeAnimation(r, g, b, 0, 1.0, 100, 0);
                break;
            default:
                _animation = new TwinkleAnimation(0, 0, 0, 0, 1.0, 100, TwinklePercent.Percent6);
                break;
        }
        
    }

    public Animation getAnimation() {
        return _animation;
    }
}