package org.frc5687.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5687.robot.commands.algae.IdleAlgae;
import org.frc5687.robot.commands.drive.TeleopDriveCommand;
import org.frc5687.robot.commands.elevator.IdleElevator;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmIO;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.HardwareAlgaeArmIO;
import org.frc5687.robot.subsystems.drive.*;
import org.frc5687.robot.subsystems.drive.modules.CTRESwerveModuleIO;
import org.frc5687.robot.subsystems.drive.modules.SimSwerveModuleIO;
import org.frc5687.robot.subsystems.drive.modules.SwerveModule;
import org.frc5687.robot.subsystems.drive.modules.SwerveModuleIO;
import org.frc5687.robot.subsystems.elevator.ElevatorIO;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.elevator.HardwareElevatorIO;
import org.frc5687.robot.subsystems.elevator.SimElevatorIO;
import org.frc5687.robot.subsystems.intake.HardwareIntakeIO;
import org.frc5687.robot.subsystems.intake.IntakeIO;
import org.frc5687.robot.util.Helpers;

public class RobotContainer {

    private final OperatorInterface _oi;
    @Logged private final DriveSubsystem _drive;
    @NotLogged private final SwerveModule[] _modules;

    private final ElevatorSubsystem _elevator;
    // @Logged
    // private final IntakeSubsystem _intake;

    private final AlgaeArmSubsystem _algaeArm;
    @Logged private final Field2d _field;

    public RobotContainer() {
        _oi = new OperatorInterface();
        _field = new Field2d();
        _modules = new SwerveModule[Constants.SwerveModule.NUM_MODULES];

        IntakeIO intakeIO =
                new HardwareIntakeIO(
                        20, 21, Constants.Intake.ROLLER_CONFIG, Constants.Intake.INTAKE_CONFIG);
        DriveIO driveIO =
                RobotBase.isSimulation()
                        ? new SimDriveIO(RobotMap.CAN.PIGEON.PIGEON)
                        : new CTREDriveIO(RobotMap.CAN.PIGEON.PIGEON, Constants.SwerveModule.CAN_BUS);

        SwerveModuleIO northWestIO;
        SwerveModuleIO northEastIO;
        SwerveModuleIO southWestIO;
        SwerveModuleIO southEastIO;
        if (RobotBase.isReal()) {
            northWestIO =
                    new CTRESwerveModuleIO(
                            Constants.DriveTrain.NW_CONFIG,
                            RobotMap.CAN.TALONFX.NORTH_WEST_TRANSLATION,
                            RobotMap.CAN.TALONFX.NORTH_WEST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_NW,
                            Constants.DriveTrain.CAN_BUS);

            northEastIO =
                    new CTRESwerveModuleIO(
                            Constants.DriveTrain.NE_CONFIG,
                            RobotMap.CAN.TALONFX.NORTH_EAST_TRANSLATION,
                            RobotMap.CAN.TALONFX.NORTH_EAST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_NE,
                            Constants.DriveTrain.CAN_BUS);

            southWestIO =
                    new CTRESwerveModuleIO(
                            Constants.DriveTrain.SW_CONFIG,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_SW,
                            Constants.DriveTrain.CAN_BUS);

            southEastIO =
                    new CTRESwerveModuleIO(
                            Constants.DriveTrain.SE_CONFIG,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_TRANSLATION,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_SE,
                            Constants.DriveTrain.CAN_BUS);
        } else {
            northWestIO =
                    new SimSwerveModuleIO(
                            Constants.DriveTrain.NW_CONFIG,
                            RobotMap.CAN.TALONFX.NORTH_WEST_TRANSLATION,
                            RobotMap.CAN.TALONFX.NORTH_WEST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_NW,
                            Constants.DriveTrain.CAN_BUS);

            northEastIO =
                    new SimSwerveModuleIO(
                            Constants.DriveTrain.NE_CONFIG,
                            RobotMap.CAN.TALONFX.NORTH_EAST_TRANSLATION,
                            RobotMap.CAN.TALONFX.NORTH_EAST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_NE,
                            Constants.DriveTrain.CAN_BUS);

            southWestIO =
                    new SimSwerveModuleIO(
                            Constants.DriveTrain.SW_CONFIG,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_TRANSLATION,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_SW,
                            Constants.DriveTrain.CAN_BUS);

            southEastIO =
                    new SimSwerveModuleIO(
                            Constants.DriveTrain.SE_CONFIG,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_TRANSLATION,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_ROTATION,
                            RobotMap.CAN.CANCODER.ENCODER_SE,
                            Constants.DriveTrain.CAN_BUS);
        }
        _modules[0] = new SwerveModule(Constants.DriveTrain.NW_CONFIG, northWestIO);
        _modules[1] = new SwerveModule(Constants.DriveTrain.NE_CONFIG, northEastIO);
        _modules[2] = new SwerveModule(Constants.DriveTrain.SW_CONFIG, southWestIO);
        _modules[3] = new SwerveModule(Constants.DriveTrain.SE_CONFIG, southEastIO);

        _drive = new DriveSubsystem(driveIO, _modules, Constants.DriveTrain.MODULE_LOCATIONS);

        // _intake = new IntakeSubsystem(intakeIO);
        if (RobotBase.isSimulation()) {
            SmartDashboard.putData("Field", _field);
        }

        ElevatorIO elevatorIO;
        if (RobotBase.isSimulation()) {
            elevatorIO = new SimElevatorIO();
        } else {
            elevatorIO =
                    new HardwareElevatorIO(
                            RobotMap.CAN.TALONFX.NORTH_WEST_ELEVATOR,
                            RobotMap.CAN.TALONFX.NORTH_EAST_ELEVATOR,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_ELEVATOR,
                            RobotMap.CAN.PIGEON.ELEVATOR);
        }

        _elevator = new ElevatorSubsystem(elevatorIO);

        AlgaeArmIO algaeArmIO = new HardwareAlgaeArmIO();
        _algaeArm = new AlgaeArmSubsystem(algaeArmIO);

        configureDefaultCommands();
        _oi.configureCommandMapping(this);
    }

    private void configureDefaultCommands() {
        // _drive.setDefaultCommand(new TeleopDriveCommand(
        //     _drive,
        //     () -> -modifyAxis(_oi.getDriverController().getLeftY()) *
        // Constants.SwerveModule.MAX_LINEAR_SPEED,
        //     () -> -modifyAxis(_oi.getDriverController().getLeftX()) *
        // Constants.SwerveModule.MAX_LINEAR_SPEED,
        //     () -> -modifyAxis(_oi.getDriverController().getRightX()) *
        // Constants.SwerveModule.MAX_ANGULAR_SPEED,
        //     () -> true  // Always field relative
        // ));

        _drive.setDefaultCommand(
                new TeleopDriveCommand(
                        _drive, () -> 0, () -> 0, () -> 0, () -> true // Always field relative
                        ));

        _elevator.setDefaultCommand(new IdleElevator(_elevator));
        // _elevator.setDefaultCommand(new SetElevatorPosition(
        //     _elevator,
        //     () -> -modifyAxis(_oi.getDriverController().getLeftY())
        // ));

        // _algaeArm.setDefaultCommand(new IntakeAlgae(_algaeArm, 0,
        // () -> -modifyAxis(_oi.getDriverController().getRightX()));
        _algaeArm.setDefaultCommand(new IdleAlgae(_algaeArm));
        //     _intake.setDefaultCommand(new IdleIntake(_intake));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void periodic() {
        if (RobotBase.isSimulation()) {
            _field.setRobotPose(_drive.getPose());
        }
    }

    // for not be lazy and just square input TODO: DONT
    private static double modifyAxis(double value) {
        value = Helpers.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    public DriveSubsystem getDrivet() {
        return _drive;
    }

    public ElevatorSubsystem getElevator() {
        return _elevator;
    }
}
