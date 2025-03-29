package org.frc5687.robot.subsystems.intake;

import org.frc5687.robot.Constants;

public enum IntakeState {
    DEPLOYED(0.00),
    CLIMB(0.01),
    DEPLOYED_UNSTICK(0.1),
    PASSOFF_TO_CORAL(Constants.Intake.MAX_ANGLE),
    IDLE(2.38);

    private final double _value;

    IntakeState(double value) {
        _value = value;
    }

    public double getValue() {
        return _value;
    }
}
