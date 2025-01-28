package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(Math.PI / 2), // 180 deg
    GROUND_PICKUP(0),
    REEF_PICKUP(3.59538), // 206 deg
    PROCESSOR_DROPOFF(2.75762); // 158 deg

    private final double _value;

    AlgaeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
