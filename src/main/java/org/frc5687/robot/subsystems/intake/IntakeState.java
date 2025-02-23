package org.frc5687.robot.subsystems.intake;

public enum IntakeState {
    DEPLOYED(0.14), // 180 deg
    STOWED(Math.PI / 4.0),
    PASSOFF_TO_CORAL(2.2), // 130 deg
    IDLE(Math.PI / 2.0),
    EJECT(0.1); // 90 deg

    private final double _value;

    IntakeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
