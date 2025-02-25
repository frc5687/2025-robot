package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(1.818), // ratchet is locked in this state :)
    GROUND_PICKUP(3.6),
    REEF_PICKUP(3.465),
    PROCESSOR_DROPOFF(2.75762),
    BARGE_DROPOFF(Math.PI / 2),
    CORAL_DODGE(2.566);

    private final double _armAngle;

    AlgaeState(double armAngle) {
        _armAngle = armAngle;
    }

    public double getArmAngle() {
        return _armAngle;
    }
}
