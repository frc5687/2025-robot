package org.frc5687.robot.subsystems.intake;

public record IntakeConfig(

boolean isRollerMotorInverted,
boolean isRollerBrakeMode,

double rollerMotorCurrentLimit,

String canBUS
) {}

    

