package org.frc5687.robot.util.sensors;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import org.frc5687.robot.util.Helpers;

public class RevBoreEncoder extends DutyCycleEncoder {
    // These are from https://www.revrobotics.com/rev-11-1271/
    private static final double OUTPUT_FREQ = 975.6;
    private static final double OUTPUT_PERIOD = 1025;
    private static final double MIN_PULSE = 1.0 / OUTPUT_PERIOD;
    private static final double MAX_PULSE = 1024.0 / OUTPUT_PERIOD;

    public RevBoreEncoder(int channel, double expectedZero) {
        super(channel, Math.PI * 2.0, expectedZero);
        setDutyCycleRange(MIN_PULSE, MAX_PULSE);
        setAssumedFrequency(OUTPUT_FREQ);
    }

    /**
     * Return the absolute angle in radians
     *
     * @return
     */
    public double getAngle() {
        return Helpers.angleWrap(get(), true);
    }
}
