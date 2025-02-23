package org.frc5687.robot.subsystems.superstructure;

import java.util.function.Supplier;

public record SuperstructureRequest(
        SuperstructureState targetPosition,
        MotorState targetMotors,
        RequestType type,
        Supplier<Boolean> driveCondition,
        String description) {
    public SuperstructureRequest {
        if (description == null) description = "Unnamed Request";
    }
}
