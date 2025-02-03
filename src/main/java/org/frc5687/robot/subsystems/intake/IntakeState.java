package org.frc5687.robot.subsystems.intake;

public enum IntakeState {
    DEPLOYED(Math.PI), // 180 deg
    STOWED(Math.PI / 4.0), // 45 deg
    PASSOFF_TO_CORAL((3.0 * Math.PI) / 4.0), // 135 deg
    IDLE(Math.PI / 2.0); // 90 deg

    private final double _value;

    IntakeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
