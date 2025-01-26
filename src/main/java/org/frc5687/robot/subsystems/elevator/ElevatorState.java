package org.frc5687.robot.subsystems.elevator;

public enum ElevatorState {
    STOWED(0.254),
    L1_CORAL_PLACING(0.251333),
    L1_ALGAE_GRAB(1.219200),
    L2_CORAL_PLACING(0.660400),
    L2_ALGAE_GRAB(1.625600),
    L3_CORAL_PLACING(1.066800),
    L4_CORAL_PLACING(1.5), // 1.7018
    BARGE_PLACING(1.5),
    FUNNEL_RECEIVE(0.327928);

    private final double _value;

    ElevatorState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}