package org.frc5687.robot.subsystems.intake;

public enum IntakeState {
    DEPLOYED(0.14), // 180 deg
    PASSOFF_TO_CORAL(2.2), // 130 deg
    IDLE(Math.PI / 2.0);

    private final double _value;

    IntakeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
