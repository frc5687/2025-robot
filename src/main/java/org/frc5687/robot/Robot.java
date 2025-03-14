// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5687.robot;

import static edu.wpi.first.units.Units.Seconds;

import au.grapplerobotics.CanBridge;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.robot.util.ReefAlignmentHelpers;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

@Logged
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    @NotLogged private final RobotContainer _robotContainer;

    public Robot() {
        // DataLogManager.start("", "", 0.1);
        SignalLogger.enableAutoLogging(false);
        Epilogue.configure(
                config -> {
                    config.root = "Robot";
                    config.minimumImportance = Logged.Importance.CRITICAL;
                    config.errorHandler = ErrorHandler.printErrorMessages();
                    config.loggingPeriod = Seconds.of(0.1);
                });
        Epilogue.bind(this);
        _robotContainer = new RobotContainer(this);
        Threads.setCurrentThreadPriority(true, 99);
        CanBridge.runTCP();

        for (int i = 1; i <= 6; i++) {
            Pose2d left = ReefAlignmentHelpers.calculateTargetPose(i, ReefSide.LEFT);
            Pose2d right = ReefAlignmentHelpers.calculateTargetPose(i, ReefSide.RIGHT);
            System.out.println(
                    String.format(
                            "Face: %d\n Left: %s\n Right: %s\n",
                            i,
                            "X: " + left.getX() + ", Y: " + left.getY() + " Heading: " + left.getRotation(),
                            "X: " + right.getX() + ", Y: " + right.getY() + " Heading: " + right.getRotation()));
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        _robotContainer.periodic();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = _robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
