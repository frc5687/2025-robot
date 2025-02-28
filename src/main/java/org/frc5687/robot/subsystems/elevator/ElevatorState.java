package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;

public enum ElevatorState {
    STOWED(Constants.Elevator.MIN_HEIGHT),
    FUNNEL_RECEIVE(Constants.Elevator.MIN_HEIGHT),
    L1_CORAL_PLACING(Constants.Elevator.MIN_HEIGHT),
    L2_CORAL_PLACING(0.128),
    L3_CORAL_PLACING(0.320),
    LOW_ALGAE_GRAB(0.355),
    HIGH_ALGAE_GRAB(0.552),
    L4_CORAL_PLACING(Constants.Elevator.MAX_HEIGHT),
    BARGE_PLACING(Constants.Elevator.MAX_HEIGHT);

    private final double _height;

    ElevatorState(double height) {
        _height = height;
    }

    public double getHeight() {
        return _height;
    }
}
