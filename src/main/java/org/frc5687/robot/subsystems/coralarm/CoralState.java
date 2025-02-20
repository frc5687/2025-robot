package org.frc5687.robot.subsystems.coralarm;

import edu.wpi.first.math.util.Units;

public enum CoralState {
    STOWED(Units.degreesToRadians(180), 0),
    IDLE_NO_CORAL(Units.degreesToRadians(180 - 117), 0), // 117 deg
    IDLE_WITH_CORAL(Math.PI / 2, 0), // 90 deg
    RECEIVE_FROM_FUNNEL(Units.degreesToRadians(210), 0),
    RECEIVE_FROM_FUNNEL_RUNNING(Units.degreesToRadians(210), 12),
    PLACING(Units.degreesToRadians(30), 0), // 27.5 deg
    RECEIVE_FROM_INTAKE(Units.degreesToRadians(45), 0),
    DURING_HANDOFF(Units.degreesToRadians(135), 12),
    PLACING_L4(Units.degreesToRadians(60), 0); // 27.5 deg

    private final double _armAngle;
    private final double _rollerVoltage;

    CoralState(double armAngle, double rollerVoltage) {
        _armAngle = armAngle;
        _rollerVoltage = rollerVoltage;
    }

    public double getArmAngle() {
        return _armAngle;
    }

    public double getRollerVoltage() {
        return _rollerVoltage;
    }
}
