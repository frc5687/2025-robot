package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;

public enum CoralState {
    STOWED(Units.degreesToRadians(270)),
    IDLE(Units.degreesToRadians(180)),
    RECEIVE_FROM_FUNNEL(Constants.CoralArm.MAX_ANGLE),
    PLACING(Units.degreesToRadians(30 + 90)),
    RECEIVE_FROM_INTAKE(Units.degreesToRadians(45 + 90)),
    DURING_HANDOFF(Units.degreesToRadians(135 + 90)),
    PLACING_L4(Units.degreesToRadians(30 + 90 + 15));

    private final double _armAngle;

    CoralState(double armAngle) {
        _armAngle = armAngle;
    }

    public double getArmAngle() {
        return _armAngle;
    }
}
