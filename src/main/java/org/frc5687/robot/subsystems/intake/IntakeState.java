package org.frc5687.robot.subsystems.intake;

public enum IntakeState {
    DEPLOYED(Math.PI), // 180 deg
    STOWED(0.785398), // 45 deg
    PASSOFF_TO_CORAL(0.785398), // 45 deg
    IDLE(Math.PI / 2); // 90 deg

    private final double _value;

    IntakeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
