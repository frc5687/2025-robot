package org.frc5687.robot.subsystems.intake;

public record IntakeConfig(
        boolean isRollerMotorInverted,
        boolean isRollerBrakeMode,
        double rollerMotorCurrentLimit,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        String canBUS) {}
