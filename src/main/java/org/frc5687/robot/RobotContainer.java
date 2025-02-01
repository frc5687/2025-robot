package org.frc5687.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5687.robot.commands.algae.IntakeAlgae;
import org.frc5687.robot.commands.elevator.IdleElevator;
import org.frc5687.robot.commands.intake.RunIntake;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmIO;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.HardwareAlgaeArmIO;
import org.frc5687.robot.subsystems.algaearm.SimAlgaeArmIO;
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
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureTracker;
import org.frc5687.robot.util.Helpers;

public class RobotContainer {

    private final Robot _robot;
    private final OperatorInterface _oi;
    //     private final DriveSubsystem _drive;
    private final SwerveModule[] _modules;

    private final ElevatorSubsystem _elevator;
    // @Logged
    private final IntakeSubsystem _intake;
    private final AlgaeArmSubsystem _algaeArm;
    //     private final CoralArmSubsystem _coralArm;

    private final SuperstructureTracker _superstructureTracker;
    private final Field2d _field;

    public RobotContainer(Robot robot) {
        _robot = robot;
        _oi = new OperatorInterface();
        _field = new Field2d();
        _modules = new SwerveModule[Constants.SwerveModule.NUM_MODULES];

        IntakeIO intakeIO = new HardwareIntakeIO(20, 21, 22);
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

        // _drive = new DriveSubsystem(driveIO, _modules, Constants.DriveTrain.MODULE_LOCATIONS);

        _intake = new IntakeSubsystem(intakeIO);
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

        AlgaeArmIO algaeArmIO =
                RobotBase.isSimulation() ? new SimAlgaeArmIO() : new HardwareAlgaeArmIO();
        _algaeArm = new AlgaeArmSubsystem(algaeArmIO);

        // CoralArmIO coralArmIO =
        //         RobotBase.isSimulation() ? new SimCoralArmIO() : new HardwareCoralArmIO();
        // _coralArm = new CoralArmSubsystem(coralArmIO);

        // _intake = new IntakeSubsystem(intakeIO);
        configureDefaultCommands();

        _superstructureTracker = new SuperstructureTracker(this);
        _oi.configureCommandMapping(this);

        // Need to control faster due to stabilization
        addElevatorControlLoop();
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

        // _drive.setDefaultCommand(
        // new TeleopDriveCommand(
        //         _drive, () -> 0, () -> 0, () -> 0, () -> true // Always field relative
        //         ));

        _elevator.setDefaultCommand(new IdleElevator(_elevator));
        // _elevator.setDefaultCommand(new SetElevatorPosition(
        //     _elevator,
        //     () -> -modifyAxis(_oi.getDriverController().getLeftY())
        // ));

        _algaeArm.setDefaultCommand(
                new IntakeAlgae(
                        _algaeArm,
                        0,
                        () -> MathUtil.clamp(-modifyAxis(_oi.getDriverController().getLeftX()), -12, 12)));
        // _algaeArm.setDefaultCommand(new IdleAlgae(_algaeArm));
        // _coralArm.setDefaultCommand(new IdleCoral(_coralArm));
        // _coralArm.setDefaultCommand(new setCoralArmAngle(_coralArm))
        // _intake.setDefaultCommand(new IdleIntake(_intake() ->
        // -modifyAxis(_oi.getDriverController().getLeft())));
        _intake.setDefaultCommand(
                new RunIntake(
                        _intake,
                        0,
                        0,
                        () -> MathUtil.clamp(-modifyAxis(_oi.getDriverController().getLeftX()), -12, 12)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void periodic() {
        // if (RobotBase.isSimulation()) {
        //     _field.setRobotPose(_drive.getPose());
        // }
    }

    public void addElevatorControlLoop() {
        _robot.addPeriodic(
                _elevator::processWithSeparateControl,
                Constants.Elevator.PERIOD,
                Constants.Elevator.PERIOD / 2.0);
    }

    // for not be lazy and just square input TODO: DONT
    private static double modifyAxis(double value) {
        value = Helpers.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    //     public DriveSubsystem getDrivet() {
    //         return _drive;
    //     }

    public ElevatorSubsystem getElevator() {
        return _elevator;
    }

    public AlgaeArmSubsystem getAlgae() {
        return _algaeArm;
    }

    //     public CoralArmSubsystem getCoral() {
    //         return _coralArm;
    //     }

    public SuperstructureTracker getSuperstructureTracker() {
        return _superstructureTracker;
    }
}
