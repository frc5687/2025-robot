package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;

public enum ElevatorState {
    STOWED(Constants.Elevator.MIN_PLATFORM_HEIGHT),
    L1_CORAL_PLACING(Constants.Elevator.MIN_PLATFORM_HEIGHT),
    L1_ALGAE_GRAB(1.269200),
    L2_CORAL_PLACING(0.460400),
    L2_ALGAE_GRAB(0.816), // Is this real?
    L3_CORAL_PLACING(.866800),
    L4_CORAL_PLACING(Constants.Elevator.MAX_PLATFORM_HEIGHT), // 1.7018
    BARGE_PLACING(Constants.Elevator.MAX_PLATFORM_HEIGHT),
    FUNNEL_RECEIVE(Constants.Elevator.MIN_PLATFORM_HEIGHT);

    private final double _height;

    ElevatorState(double height) {
        _height = height;
    }

    public double getHeight() {
        return _height;
    }
}
