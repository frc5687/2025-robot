package org.frc5687.robot.subsystems.algaearm;

public enum AlgaeState {
    IDLE(2.312), // wheel is not touching in this state
    GROUND_PICKUP(3.7),
    REEF_PICKUP(3.5),
    PROCESSOR_DROPOFF(3.14),
    BARGE_DROPOFF(1.575),
    CORAL_DODGE(2.566);

    private final double _armAngle;

    AlgaeState(double armAngle) {
        _armAngle = armAngle;
    }

    public double getArmAngle() {
        return _armAngle;
    }
}
