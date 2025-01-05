// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5687.robot;

import org.frc5687.robot.subsystems.drive.DriveIO;
import org.frc5687.robot.subsystems.drive.DriveIOCTRE;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final DriveSubsystem _drive;

  public RobotContainer() {
    Pigeon2 imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, "CANivore"); // TODO: real canbus
    DriveIO driveIO = new DriveIOCTRE(imu);

    _drive = new DriveSubsystem(driveIO);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
