package org.frc5687.robot.subsystems.drive.modules;

import org.frc5687.robot.util.PIDConstants;

public record SwerveModuleConfig(
        String moduleName,
        double wheelRadius,
        double driveGearRatio,
        double steerGearRatio,
        double couplingRatio,
        PIDConstants drivePID,
        double driveKs,
        double driveKv,
        double driveKa,
        double driveCurrentLimit,
        PIDConstants steerPID,
        double steerKs,
        double steerKv,
        double steerKa,
        double steerCurrentLimit,
        double steerMotionCruiseVelocity,
        double steerMotionAcceleration,
        double absoluteEncoderOffset,
        boolean driveInverted,
        boolean steerInverted) {}
