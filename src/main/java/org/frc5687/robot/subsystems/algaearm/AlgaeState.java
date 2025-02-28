package org.frc5687.robot.subsystems.algaearm;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;

public enum AlgaeState {
    IDLE(1.818), // ratchet is locked in this state :)
    GROUND_PICKUP(3.7),
    REEF_PICKUP(3.7),
    PROCESSOR_DROPOFF(2.75762),
    BARGE_DROPOFF(Constants.AlgaeArm.TOP_EJECT_SAFE_ANGLE - Units.degreesToRadians(5)),
    CORAL_DODGE(2.566);

    private final double _armAngle;

    AlgaeState(double armAngle) {
        _armAngle = armAngle;
    }

    public double getArmAngle() {
        return _armAngle;
    }
}
