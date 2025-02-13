package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;

public enum ElevatorState {
    STOWED(Constants.Elevator.MIN_PLATFORM_HEIGHT),
    L1_CORAL_PLACING(0.5),
    L1_ALGAE_GRAB(1.219200),
    L2_CORAL_PLACING(0.660400),
    L2_ALGAE_GRAB(1.066800), // Is this real?
    L3_CORAL_PLACING(1.066800),
    L4_CORAL_PLACING(Constants.Elevator.MAX_PLATFORM_HEIGHT), // 1.7018
    BARGE_PLACING(Constants.Elevator.MAX_PLATFORM_HEIGHT),
    FUNNEL_RECEIVE(0.327928);

    private final double _value;

    ElevatorState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
