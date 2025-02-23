package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(Math.PI / 2, 0), // 90 deg
    IDLE_WITH_ALGAE(Math.PI / 2, 0),
    IDLE_WITH_ALGAE_WHEEL(Math.PI / 2, 0),
    GROUND_PICKUP(3.5, 12),
    REEF_PICKUP_NO_WHEELS(Math.PI, 0),
    REEF_PICKUP_WHEEL(2.456, 12), // 180 deg
    PROCESSOR_DROPOFF(2.75762, 0),
    PROCESSOR_DROPOFF_WHEEL(2.75762, -12); // 158 deg

    private final double _armAngle;
    private final double _wheelVoltage;

    AlgaeState(double armAngle, double wheelVoltage) {
        _armAngle = armAngle;
        _wheelVoltage = wheelVoltage;
    }

    public double getArmAngle() {
        return _armAngle;
    }

    public double getWheelVoltage() {
        return _wheelVoltage;
    }
}
