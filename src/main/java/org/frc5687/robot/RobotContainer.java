package org.frc5687.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.robot.commands.algae.IdleAlgae;
import org.frc5687.robot.commands.coral.IdleCoral;
import org.frc5687.robot.commands.drive.TeleopDriveCommand;
import org.frc5687.robot.commands.elevator.IdleElevator;
import org.frc5687.robot.commands.intake.IdleIntake;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmIO;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.HardwareAlgaeArmIO;
import org.frc5687.robot.subsystems.algaearm.SimAlgaeArmIO;
import org.frc5687.robot.subsystems.climber.ClimberIO;
import org.frc5687.robot.subsystems.climber.ClimberSubsystem;
import org.frc5687.robot.subsystems.climber.HardwareClimberArmIO;
import org.frc5687.robot.subsystems.climber.SimClimberIO;
import org.frc5687.robot.subsystems.coralarm.CoralArmIO;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.coralarm.HardwareCoralArmIO;
import org.frc5687.robot.subsystems.coralarm.SimCoralArmIO;
import org.frc5687.robot.subsystems.drive.CTREDriveIO;
import org.frc5687.robot.subsystems.drive.DriveIO;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.drive.SimDriveIO;
import org.frc5687.robot.subsystems.elevator.ElevatorIO;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.elevator.HardwareElevatorIO;
import org.frc5687.robot.subsystems.elevator.SimElevatorIO;
import org.frc5687.robot.subsystems.intake.IntakeIO;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.subsystems.intake.SimIntakeIO;
import org.frc5687.robot.subsystems.superstructure.RequestType;
import org.frc5687.robot.subsystems.superstructure.SuperstructureGoals;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.vision.PhotonVisionIO;
import org.frc5687.robot.subsystems.vision.SimVisionIO;
import org.frc5687.robot.subsystems.vision.VisionIO;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.util.QuestNav;

public class RobotContainer implements EpilogueLog {

    private final Robot _robot;
    private final OperatorInterface _oi;
    private final DriveSubsystem _drive;
    private final ElevatorSubsystem _elevator;
    private final IntakeSubsystem _intake;
    private final AlgaeArmSubsystem _algaeArm;
    private final CoralArmSubsystem _coralArm;
    private final ClimberSubsystem _climber;

    private final VisionSubsystem _vision;

    private final SuperstructureManager _superstructureManager;

    private SendableChooser<Command> _autoChooser;

    private final QuestNav _questNav;
    private final Field2d _field;
    private boolean _isCoralMode;

    public RobotContainer(Robot robot) {
        _robot = robot;
        _oi = new OperatorInterface();
        _field = new Field2d();
        _questNav = new QuestNav();
        DriveIO driveIO =
                RobotBase.isSimulation()
                        ? new SimDriveIO(RobotMap.CAN.PIGEON.PIGEON)
                        : new CTREDriveIO(RobotMap.CAN.PIGEON.PIGEON, Constants.SwerveModule.CAN_BUS);

        _drive = new DriveSubsystem(this, driveIO, Constants.DriveTrain.MODULE_LOCATIONS);

        RobotStateManager.getInstance()
                .initEstimators(_drive::getModulePositions, _drive::getHeading, _questNav);

        ElevatorIO elevatorIO;
        if (RobotBase.isSimulation()) {
            elevatorIO = new SimElevatorIO();
        } else {
            elevatorIO =
                    new HardwareElevatorIO(
                            RobotMap.CAN.TALONFX.NORTH_WEST_ELEVATOR,
                            RobotMap.CAN.TALONFX.NORTH_EAST_ELEVATOR,
                            RobotMap.CAN.LASERCAN.LASERCAN);
        }

        _elevator = new ElevatorSubsystem(this, elevatorIO);

        AlgaeArmIO algaeArmIO =
                RobotBase.isSimulation() ? new SimAlgaeArmIO() : new HardwareAlgaeArmIO();
        _algaeArm = new AlgaeArmSubsystem(this, algaeArmIO);

        CoralArmIO coralArmIO =
                RobotBase.isSimulation() ? new SimCoralArmIO() : new HardwareCoralArmIO();
        _coralArm = new CoralArmSubsystem(this, coralArmIO);

        IntakeIO intakeIO = /*  RobotBase.isSimulation() ?*/
                new SimIntakeIO() /*  : new HardwareIntakeIO() */;
        _intake = new IntakeSubsystem(this, intakeIO);

        ClimberIO climberIO =
                RobotBase.isSimulation() ? new SimClimberIO() : new HardwareClimberArmIO();
        _climber = new ClimberSubsystem(this, climberIO);

        VisionIO visionIO = RobotBase.isSimulation() ? new SimVisionIO() : new PhotonVisionIO();
        _vision = new VisionSubsystem(this, visionIO);

        _superstructureManager = new SuperstructureManager(this);

        _oi.configureCommandMapping(this);

        _isCoralMode = true;
        setupNamedCommand();
        configureDefaultCommands();
    }

    private void configureDefaultCommands() {
        _drive.setDefaultCommand(
                new TeleopDriveCommand(
                        _drive,
                        () ->
                                -modifyAxis(_oi.getDriverController().getLeftY())
                                        * Constants.SwerveModule.MAX_LINEAR_SPEED,
                        () ->
                                -modifyAxis(_oi.getDriverController().getLeftX())
                                        * Constants.SwerveModule.MAX_LINEAR_SPEED,
                        () ->
                                -modifyAxis(_oi.getDriverController().getRightX())
                                        * Constants.SwerveModule.MAX_ANGULAR_SPEED,
                        () -> true // Always field relative
                        ));

        // _drive.setDefaultCommand(
        // new TeleopDriveCommand(
        //         _drive, () -> 0, () -> 0, () -> 0, () -> true // Always field relative
        //         ));

        _elevator.setDefaultCommand(new IdleElevator(_elevator));
        // _elevator.setDefaultCommand(new SetElevatorPosition(
        //     _elevator,
        //     () -> -modifyAxis(_oi.getDriverController().getLeftY())
        // ));

        _algaeArm.setDefaultCommand(new IdleAlgae(_algaeArm));
        // _algaeArm.setDefaultCommand(new IdleAlgae(_algaeArm));
        _coralArm.setDefaultCommand(new IdleCoral(_coralArm));
        // _coralArm.setDefaultCommand(new setCoralArmAngle(_coralArm))
        _intake.setDefaultCommand(new IdleIntake(_intake));
        // _intake.setDefaultCommand(
        //         new RunIntake(
        //                 _intake,
        //                 0,
        //                 0,
        //                 () -> MathUtil.clamp(-modifyAxis(_oi.getDriverController().getLeftX()), -12,
        // 12)));
        _autoChooser = AutoBuilder.buildAutoChooser("test");
        SmartDashboard.putData("Auto Chooser", _autoChooser);
    }

    public Command getAutonomousCommand() {
        return _autoChooser.getSelected();
    }

    private void setupNamedCommand() {
        NamedCommands.registerCommand(
                "CoralL4",
                _superstructureManager.setToPlaceHeight(
                        SuperstructureGoals.PLACE_CORAL_L4, RequestType.IMMEDIATE));

        if (RobotBase.isSimulation()) {
            // NamedCommands.registerCommand( "ReceiveFunnel",
            // SuperstructureFactory.receiveFromFunnelSim(this));

            NamedCommands.registerCommand(
                    "ReceiveFunnel", _superstructureManager.receiveFunnelSim(RequestType.IMMEDIATE));
        } else {
            NamedCommands.registerCommand(
                    "ReceiveFunnel", _superstructureManager.receiveFunnel(RequestType.IMMEDIATE));
        }

        NamedCommands.registerCommand(
                "CoralL4",
                _superstructureManager.setToPlaceHeight(
                        SuperstructureGoals.PLACE_CORAL_L4, RequestType.IMMEDIATE));

        NamedCommands.registerCommand(
                "CoralL3",
                _superstructureManager.setToPlaceHeight(
                        SuperstructureGoals.PLACE_CORAL_L3, RequestType.IMMEDIATE));

        NamedCommands.registerCommand(
                "CoralL2",
                _superstructureManager.setToPlaceHeight(
                        SuperstructureGoals.PLACE_CORAL_L2, RequestType.IMMEDIATE));

        // NamedCommands.registerCommand(
        //         "Place", _superstructureManager.placeAtCurrentHeight(RequestType.IMMEDIATE));

        // Supplier<Boolean> falseSupplier =
        //         () -> {
        //             return false;
        //         };
        // NamedCommands.registerCommand(
        //         "CoralL4", SuperstructureFactory.placeCoralL4(this, false, falseSupplier));
        // NamedCommands.registerCommand(
        //         "CoralL3", SuperstructureFactory.placeCoralL3(this, false, falseSupplier));
        // NamedCommands.registerCommand(
        //         "CoralL2", SuperstructureFactory.placeCoralL2(this, falseSupplier));
        // NamedCommands.registerCommand("Place", SuperstructureFactory.place(this));
        // place shoots it out
    }

    public void periodic() {
        _questNav.timeSinceLastUpdate();
        RobotStateManager.getInstance().logComponentPoses();
        RobotStateManager.getInstance().updateOdometry();
        RobotStateManager.getInstance().logEstimatedPoses();
    }

    // for not be lazy and just square input TODO: DONT
    private static double modifyAxis(double value) {
        value = Helpers.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    public DriveSubsystem getDrive() {
        return _drive;
    }

    public ElevatorSubsystem getElevator() {
        return _elevator;
    }

    public AlgaeArmSubsystem getAlgae() {
        return _algaeArm;
    }

    public CoralArmSubsystem getCoral() {
        return _coralArm;
    }

    public IntakeSubsystem getIntake() {
        return _intake;
    }

    public ClimberSubsystem getClimber() {
        return _climber;
    }

    public VisionSubsystem getVision() {
        return _vision;
    }

    public QuestNav getQuestNav() {
        return _questNav;
    }

    public void setIntakeMode() {
        _isCoralMode = !_isCoralMode;
    }

    public boolean getIntakeMode() {
        return _isCoralMode;
    }

    public SuperstructureManager getSuperstructureManager() {
        return _superstructureManager;
    }

    @Override
    public String getLogBase() {
        return "RobotContainer";
    }
}
