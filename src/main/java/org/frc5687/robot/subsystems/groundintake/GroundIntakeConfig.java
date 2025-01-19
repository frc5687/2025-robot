package org.frc5687.robot.subsystems.groundintake;

import org.frc5687.robot.util.PIDConstants;

public record GroundIntakeConfig(

String canBUS,
double pivotGearRatio,

PIDConstants rightRollerPID,
    double rightRollerKs,
    double rightRollerKv,
    double rightRollerKa,
    double rightRollerCurrentLimit,

    PIDConstants leftRollerPID,
    double leftRollerKs,
    double leftRollerKv,
    double leftRollerKa,
    double leftRollerCurrentLimit,

    PIDConstants pivotPID,
    double pivotKs,
    double pivotKv,
    double pivotKa,
    double pivotCurrentLimit,
    double pivotMotionCruiseVelocity,
    double pivotMotionAcceleration,

    double absoluteEncoderOffset,
    boolean rightRollerInverted,
    boolean leftRollerInverted,
    boolean pivotInverted
) {}
