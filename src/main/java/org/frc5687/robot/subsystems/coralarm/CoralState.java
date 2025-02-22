package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.util.Units;

public enum CoralState {
    STOWED(Units.degreesToRadians(270), 0),
    IDLE_NO_CORAL(Units.degreesToRadians(270), 0), // 117 deg
    IDLE_WITH_CORAL(Units.degreesToRadians(180), 0), // 90 deg
    RECEIVE_FROM_FUNNEL(Units.degreesToRadians(220 + 90), 0),
    RECEIVE_FROM_FUNNEL_RUNNING(Units.degreesToRadians(220 + 90), 12),
    PLACING(Units.degreesToRadians(30 + 90), 0), // 27.5 deg
    RECEIVE_FROM_INTAKE(Units.degreesToRadians(45 + 90), 0),
    DURING_HANDOFF(Units.degreesToRadians(135 + 90), 12),
    PLACING_L4(Units.degreesToRadians(30 + 90 + 15), 0); // 27.5 deg

    private final double _armAngle;
    private final double _rollerDutyCycle;

    CoralState(double armAngle, double rollerDutyCycle) {
        _armAngle = armAngle;
        _rollerDutyCycle = rollerDutyCycle;
    }

    public double getArmAngle() {
        return _armAngle;
    }

    public double getRollerDutyCycle() {
        return _rollerDutyCycle;
    }
}
