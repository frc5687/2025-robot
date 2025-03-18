package org.frc5687.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.robot.commands.algae.IdleAlgae;
import org.frc5687.robot.commands.auto.AutoActions;
import org.frc5687.robot.commands.coral.IdleCoral;
import org.frc5687.robot.commands.drive.DynamicDriveToReefBranch;
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
import org.frc5687.robot.subsystems.lights.HardwareLightsIO;
import org.frc5687.robot.subsystems.lights.LightSubsystem;
import org.frc5687.robot.subsystems.superstructure.RequestType;
import org.frc5687.robot.subsystems.superstructure.SuperstructureManager;
import org.frc5687.robot.subsystems.vision.LimelightVisionIO;
import org.frc5687.robot.subsystems.vision.SimVisionIO;
import org.frc5687.robot.subsystems.vision.VisionIO;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.util.QuestNav;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class RobotContainer implements EpilogueLog {

    private final Robot _robot;
    private final OperatorInterface _oi;
    private final DriveSubsystem _drive;
    private final ElevatorSubsystem _elevator;
    private final IntakeSubsystem _intake;
    private final AlgaeArmSubsystem _algaeArm;
    private final CoralArmSubsystem _coralArm;
    private final ClimberSubsystem _climber;
    private final LightSubsystem _lights;

    private final VisionSubsystem _vision;

    private final SuperstructureManager _superstructureManager;

    private SendableChooser<Command> _autoChooser;

    private final QuestNav _questNav;

    public RobotContainer(Robot robot) {
        _robot = robot;
        _oi = new OperatorInterface();
        _questNav = new QuestNav();

        // TODO implement simulation io
        _lights = new LightSubsystem(this, new HardwareLightsIO());

        DriveIO driveIO =
                RobotBase.isSimulation()
                        ? new SimDriveIO(RobotMap.CAN.PIGEON.PIGEON)
                        : new CTREDriveIO(RobotMap.CAN.PIGEON.PIGEON, Constants.SwerveModule.CAN_BUS);

        _drive = new DriveSubsystem(this, driveIO, Constants.DriveTrain.MODULE_LOCATIONS);

        VisionIO visionIO = RobotBase.isSimulation() ? new SimVisionIO() : new LimelightVisionIO();
        _vision = new VisionSubsystem(this, visionIO);

        RobotStateManager.getInstance()
                .initEstimators(
                        _drive::getModulePositions,
                        _drive::getHeading,
                        _drive::getMeasuredChassisSpeeds,
                        _vision,
                        _questNav);

        ElevatorIO elevatorIO;
        if (RobotBase.isSimulation()) {
            elevatorIO = new SimElevatorIO();
        } else {
            elevatorIO =
                    new HardwareElevatorIO(
                            RobotMap.CAN.TALONFX.NORTH_WEST_ELEVATOR,
                            RobotMap.CAN.TALONFX.NORTH_EAST_ELEVATOR,
                            RobotMap.CAN.LASERCAN.ELEVATOR);
        }

        _elevator = new ElevatorSubsystem(this, elevatorIO);

        AlgaeArmIO algaeArmIO =
                RobotBase.isSimulation() ? new SimAlgaeArmIO() : new HardwareAlgaeArmIO();
        _algaeArm = new AlgaeArmSubsystem(this, algaeArmIO);

        CoralArmIO coralArmIO =
                RobotBase.isSimulation() ? new SimCoralArmIO() : new HardwareCoralArmIO();
        _coralArm = new CoralArmSubsystem(this, coralArmIO);

        IntakeIO intakeIO =
                // RobotBase.isSimulation() ?
                new SimIntakeIO();
        //   : new HardwareIntakeIO();
        _intake = new IntakeSubsystem(this, intakeIO);

        ClimberIO climberIO =
                RobotBase.isSimulation() ? new SimClimberIO() : new HardwareClimberArmIO();
        _climber = new ClimberSubsystem(this, climberIO);

        _superstructureManager = new SuperstructureManager(this);

        _oi.configureCommandMapping(this);

        setupNamedCommand();
        configureDefaultCommands();

        _autoChooser = AutoBuilder.buildAutoChooser("test");
        SmartDashboard.putData("Auto Chooser", _autoChooser);
    }

    private void configureDefaultCommands() {
        _drive.setDefaultCommand(
                new TeleopDriveCommand(
                        _drive,
                        () -> -modifyAxis(_oi.getDriverController().getLeftY()) * Constants.DriveTrain.MAX_MPS,
                        () -> -modifyAxis(_oi.getDriverController().getLeftX()) * Constants.DriveTrain.MAX_MPS,
                        () -> -modifyAxis(_oi.getDriverController().getRightX()) * Constants.DriveTrain.MAX_MPS,
                        () -> true // Always field relative
                        ));

        _elevator.setDefaultCommand(new IdleElevator(_elevator));
        _algaeArm.setDefaultCommand(new IdleAlgae(_algaeArm));
        _coralArm.setDefaultCommand(new IdleCoral(_coralArm));
        _intake.setDefaultCommand(new IdleIntake(_intake));
    }

    public Command getAutonomousCommand() {
        return _autoChooser.getSelected();
    }

    private void setupNamedCommand() {
        // if (RobotBase.isSimulation()) {
        //     NamedCommands.registerCommand(
        //             "ReceiveFunnel",
        //             _superstructureManager
        //                     .receiveFunnelSim(RequestType.IMMEDIATE)
        //                     .andThen(new WaitCommand(1)));
        // } else {
        NamedCommands.registerCommand(
                "ReceiveFunnel", _superstructureManager.receiveFunnel(RequestType.IMMEDIATE));
        // }

        NamedCommands.registerCommand(
                "ReadyFunnel", _superstructureManager.receiveFunnelSim(RequestType.IMMEDIATE));

        NamedCommands.registerCommand(
                "LowAlgaeIntake",
                _superstructureManager
                        .algaeIntake(Constants.SuperstructureGoals.LOW_ALGAE_GRAB)
                        .alongWith(
                                new DynamicDriveToReefBranch(
                                        getDrive(), getSuperstructureManager(), ReefSide.ALGAE)));

        NamedCommands.registerCommand(
                "HighAlgaeIntake",
                _superstructureManager
                        .algaeIntake(Constants.SuperstructureGoals.HIGH_ALGAE_GRAB)
                        .alongWith(
                                new DynamicDriveToReefBranch(
                                        getDrive(), getSuperstructureManager(), ReefSide.ALGAE)));

        NamedCommands.registerCommand(
                "CoralL4",
                _superstructureManager.createRequest(
                        Constants.SuperstructureGoals.PLACE_CORAL_L4, RequestType.AUTO_SEQUENCE));

        NamedCommands.registerCommand(
                "ForceCoralL4",
                _superstructureManager.createRequest(
                        Constants.SuperstructureGoals.PLACE_CORAL_L4, RequestType.IMMEDIATE));

        NamedCommands.registerCommand(
                "CoralL3",
                _superstructureManager.createRequest(
                        Constants.SuperstructureGoals.PLACE_CORAL_L3, RequestType.IMMEDIATE));

        NamedCommands.registerCommand(
                "CoralL2",
                _superstructureManager.createRequest(
                        Constants.SuperstructureGoals.PLACE_CORAL_L2, RequestType.AUTO_SEQUENCE));

        NamedCommands.registerCommand("AutoPlace", AutoActions.autoPlace(this));
    }

    public void periodic() {
        _questNav.timeSinceLastUpdate();
        RobotStateManager.getInstance().logComponentPoses();
        RobotStateManager.getInstance().updateOdometry();
        RobotStateManager.getInstance().logEstimatedPoses();
    }

    public static double modifyAxis(double value) {
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

    public SuperstructureManager getSuperstructureManager() {
        return _superstructureManager;
    }

    @Override
    public String getLogBase() {
        return "RobotContainer";
    }
}
