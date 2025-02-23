package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;

/** All heights are in motor meters, NOT PLATFORM HEIGHTS. */
public enum ElevatorState {
    STOWED(Constants.Elevator.MIN_HEIGHT),
    L1_CORAL_PLACING(Constants.Elevator.MIN_HEIGHT),
    FUNNEL_RECEIVE(Constants.Elevator.MIN_HEIGHT),
    L1_ALGAE_GRAB(0.11),
    L2_CORAL_PLACING(0.11),
    L2_ALGAE_GRAB(0.31),
    L3_CORAL_PLACING(0.31),
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
