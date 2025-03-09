package org.frc5687.robot.subsystems.intake;

public enum IntakeState {
    DEPLOYED(0.0),
    PASSOFF_TO_CORAL(2.916),
    IDLE(2.38);

    private final double _value;

    IntakeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
