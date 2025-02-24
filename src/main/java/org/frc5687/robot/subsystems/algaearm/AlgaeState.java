package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(1.818), // ratchet is locked in this state :) TODO ask willette
    GROUND_PICKUP(3.5),
    REEF_PICKUP(Math.PI),
    PROCESSOR_DROPOFF(2.75762);

    private final double _armAngle;

    AlgaeState(double armAngle) {
        _armAngle = armAngle;
    }

    public double getArmAngle() {
        return _armAngle;
    }
}
