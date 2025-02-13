package org.frc5687.robot.subsystems.intake;

public enum IntakeState {
    DEPLOYED(0), // 180 deg
    STOWED(Math.PI / 4.0), // 45 deg 0.5, 2.67
    PASSOFF_TO_CORAL(2.29), // 135 deg
    IDLE(Math.PI / 2.0); // 90 deg

    private final double _value;

    IntakeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
