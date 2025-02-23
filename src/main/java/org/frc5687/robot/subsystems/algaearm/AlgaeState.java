package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(Math.PI / 2),
    GROUND_PICKUP(3.5),
    REEF_PICKUP(2.5),
    PROCESSOR_DROPOFF(2.75762);

    private final double _value;

    AlgaeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
