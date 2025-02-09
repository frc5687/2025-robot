package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(Math.PI / 2), // 90 deg
    IDLE_WITH_ALGAE(1.793),
    GROUND_PICKUP(0),
    REEF_PICKUP(Math.PI), // 180 deg
    PROCESSOR_DROPOFF(2.75762); // 158 deg

    private final double _value;

    AlgaeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
