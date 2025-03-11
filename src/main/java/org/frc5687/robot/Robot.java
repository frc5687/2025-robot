// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.frc5687.robot;

import static edu.wpi.first.units.Units.Seconds;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.ReefAlignmentHelpers;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

@Logged
public class Robot extends TimedRobot implements EpilogueLog {
    private static final double LOOP_OVERRUN_WARNING_TIMEOUT = 0.2;
    private static final double LOW_BATTERY_VOLTAGE = 11.8;
    private static final int LOG_INTERVAL = 10;

    private double _lastLoopTime = 0;
    private double _maxLoopTime = 0;
    private double _accumulatedLoopTime = 0;
    private int _loopCount = 0;

    private Command m_autonomousCommand;
    @NotLogged private RobotContainer _robotContainer;

    public Robot() {
        Threads.setCurrentThreadPriority(true, 20);

        configureLogging();
        configureWatchdog(); // taken for 6328

        RobotController.setBrownoutVoltage(6.0);
        CanBridge.runTCP();

        for (int i = 1; i <= 6; i++) {
            System.out.println(
                    String.format(
                            "Face: %d\n Left: %s\n Right: %s\n",
                            i,
                            ReefAlignmentHelpers.calculateTargetPose(i, ReefSide.LEFT),
                            ReefAlignmentHelpers.calculateTargetPose(i, ReefSide.RIGHT)));
        }

        _robotContainer = new RobotContainer(this);
    }

    private void configureLogging() {
        DataLogManager.start("", "", 0.25);
        Epilogue.configure(
                config -> {
                    config.root = "Robot";
                    config.minimumImportance = Logged.Importance.DEBUG;
                    config.errorHandler = ErrorHandler.printErrorMessages();
                    config.loggingPeriod = Seconds.of(0.25);
                });
        Epilogue.bind(this);
    }

    private void configureWatchdog() {
        try {
            Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchdogField.setAccessible(true);
            Watchdog watchdog = (Watchdog) watchdogField.get(this);
            watchdog.setTimeout(LOOP_OVERRUN_WARNING_TIMEOUT);
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to adjust loop overrun warnings.", false);
        }
    }

    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true); // TODO might want this not sure

        if (RobotBase.isSimulation()) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        }
    }

    @Override
    public void robotPeriodic() {
        double currentTime = Timer.getFPGATimestamp();
        double loopTime = currentTime - _lastLoopTime;
        _lastLoopTime = currentTime;

        _maxLoopTime = Math.max(_maxLoopTime, loopTime);
        _accumulatedLoopTime += loopTime;
        _loopCount++;

        CommandScheduler.getInstance().run();

        _robotContainer.periodic();

        if (_loopCount % LOG_INTERVAL == 0) {
            log("LoopTime/Current", loopTime);
            log("LoopTime/Max", _maxLoopTime);
            log("LoopTime/Average", (_accumulatedLoopTime / _loopCount));
            log("Battery/Voltage", RobotController.getBatteryVoltage());

            if (RobotController.getBatteryVoltage() <= LOW_BATTERY_VOLTAGE
                    && !DriverStation.isEnabled()) {
                DriverStation.reportWarning("Low battery voltage detected!", false);
            }
        }
    }

    @Override
    public void disabledInit() {
        _maxLoopTime = 0;
        _accumulatedLoopTime = 0;
        _loopCount = 0;
    }

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

    @Override
    public String getLogBase() {
        return "Robot";
    }
}
