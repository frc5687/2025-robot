package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.Constants;

public enum ElevatorState {
    STOWED(Constants.Elevator.MIN_HEIGHT),
    FUNNEL_RECEIVE(0.00),
    GROUND_INTAKE_RECEIVE(0.0208),
    L1_CORAL_PLACING(0.035),
    PROCESSOR_PLACING(Constants.Elevator.MIN_HEIGHT),
    L2_CORAL_PLACING(0.128),
    L3_CORAL_PLACING(0.320 + .02),
    LOW_ALGAE_GRAB(0.324),
    HIGH_ALGAE_GRAB(0.561 - 0.04),
    L4_CORAL_PLACING(Constants.Elevator.MAX_HEIGHT),
    BARGE_PLACING(0.65);

    private final double _height;

    ElevatorState(double height) {
        _height = height;
    }

    public double getHeight() {
        return _height;
    }
}
