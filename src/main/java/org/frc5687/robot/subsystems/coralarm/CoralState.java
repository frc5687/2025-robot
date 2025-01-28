package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.util.Units;

public enum CoralState {
    STOWED(Math.PI / 2.0),
    IDLE_NO_CORAL(Units.degreesToRadians(180 - 117)), // 117 deg
    IDLE_WITH_CORAL(Math.PI / 2), // 90 deg
    RECEIVE_FROM_FUNNEL(0),
    PLACING(Units.degreesToRadians(180 - 27.5)), // 27.5 deg
    RECEIVE_FROM_INTAKE(Units.degreesToRadians(45)); // 45 deg

    private final double value;

    CoralState(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}
