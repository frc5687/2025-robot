package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.util.Units;
import org.frc5687.robot.Constants;

public enum CoralState {
    IDLE(Units.degreesToRadians(30 + 90)),
    RECEIVE_FROM_FUNNEL(Constants.CoralArm.MAX_ANGLE),
    RECEIVE_FROM_GROUND_INTAKE(4.55),
    PLACING_L1(4.4),
    PLACING(Units.degreesToRadians(30 + 90)), // 27.5 deg
    PLACING_L4(Units.degreesToRadians(30 + 90 + 10)); // 27.5 deg

    private final double _armAngle;

    CoralState(double armAngle) {
        _armAngle = armAngle;
    }

    public double getArmAngle() {
        return _armAngle;
    }
}
