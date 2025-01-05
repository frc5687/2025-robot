package org.frc5687.robot;

import org.frc5687.robot.commands.drive.DriveTeleop;
import org.frc5687.robot.subsystems.drive.DriveIO;
import org.frc5687.robot.subsystems.drive.DriveIOCTRE;
import org.frc5687.robot.subsystems.drive.DriveIOSim;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    private final OperatorInterface _oi;

    private final DriveSubsystem _drive;
    private final DriveTeleop _driveTeleop; 
    
    private final Field2d _field;

    public RobotContainer() {
        _oi = new OperatorInterface();
        _field = new Field2d();

        DriveIO _driveIO;
        if (RobotBase.isSimulation()) {
            _driveIO = new DriveIOSim();
            SmartDashboard.putData("Field", _field);
        } else {
            Pigeon2 imu = new Pigeon2(RobotMap.CAN.PIGEON.PIGEON, Constants.DriveTrain.CAN_BUS);
            _driveIO = new DriveIOCTRE(imu);
        }

        _drive = new DriveSubsystem(_driveIO);
        _driveTeleop = new DriveTeleop(
            _drive, 
            () -> -_oi.getDriverController().getLeftY(),
            () -> -_oi.getDriverController().getLeftX(),
            () -> -_oi.getDriverController().getRightX(),
            false
        );
        
        configureDefaultCommands();
        _oi.configureCommandMapping(_drive);
    }

    private void configureDefaultCommands() {
        _drive.setDefaultCommand(_driveTeleop);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     */
    public void periodic() {
        if (RobotBase.isSimulation()) {
            Logger.recordOutput("Module States",  _drive.getDriveIOInputs().measuredStates);
        }
    }
}