package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(Math.PI / 2, 0), // 90 deg
    IDLE_WITH_ALGAE(1.793, 0),
    GROUND_PICKUP(3.5, -12),
    REEF_PICKUP_NO_WHEELS(Math.PI, 0),
    REEF_PICKUP_WHEEL(Math.PI, -12), // 180 deg
    PROCESSOR_DROPOFF(2.75762, 0),
    PROCESSOR_DROPOFF_WHEEL(2.75762, 12); // 158 deg

    private final double _value;
    private final double _wheelVoltage;

    AlgaeState(double value, double wheelVoltage) {
        _value = value;
        _wheelVoltage = wheelVoltage;
    }

    public double getValue() {
        return _value;
    }

    public double getWheelVoltage() {
        return _wheelVoltage;
    }
}
