package org.frc5687.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.robot.commands.drive.TeleopDriveCommand;
import org.frc5687.robot.subsystems.drive.CTREDriveIO;
import org.frc5687.robot.subsystems.drive.DriveIO;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.drive.SimDriveIO;
import org.frc5687.robot.subsystems.lights.HardwareLightsIO;
import org.frc5687.robot.subsystems.lights.LightSubsystem;
import org.frc5687.robot.subsystems.vision.HardwareVisionIO;
import org.frc5687.robot.subsystems.vision.SimVisionIO;
import org.frc5687.robot.subsystems.vision.VisionIO;
import org.frc5687.robot.subsystems.vision.VisionSubsystem;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.Helpers;

public class RobotContainer implements EpilogueLog {

    private final Robot _robot;
    private final OperatorInterface _oi;
    private final DriveSubsystem _drive;
    private final LightSubsystem _lights;
    private final VisionSubsystem _vision;

    private SendableChooser<Command> _autoChooser;

    public RobotContainer(Robot robot) {
        _robot = robot;
        _oi = new OperatorInterface();

        _lights = new LightSubsystem(this, new HardwareLightsIO());

        DriveIO driveIO =
                RobotBase.isSimulation()
                        ? new SimDriveIO(RobotMap.CAN.PIGEON.PIGEON)
                        : new CTREDriveIO(RobotMap.CAN.PIGEON.PIGEON, Constants.SwerveModule.CAN_BUS);

        _drive = new DriveSubsystem(this, driveIO, Constants.DriveTrain.MODULE_LOCATIONS);

        VisionIO visionIO = RobotBase.isSimulation() ? new SimVisionIO() : new HardwareVisionIO();
        _vision = new VisionSubsystem(this, visionIO);

        RobotStateManager.getInstance()
                .initEstimators(
                        _drive::getModulePositions, _drive::getHeading, _drive::getMeasuredChassisSpeeds);

        configureDefaultCommands();
        _oi.configureCommandMapping(this);

        setupNamedCommand();

        _autoChooser = AutoBuilder.buildAutoChooser("test");
        SmartDashboard.putData("Auto Chooser", _autoChooser);
    }

    private void configureDefaultCommands() {
        _drive.setDefaultCommand(
                new TeleopDriveCommand(
                        _drive,
                        () -> -modifyAxis(_oi.getDriverController().getLeftY()) * Constants.DriveTrain.MAX_MPS,
                        () -> -modifyAxis(_oi.getDriverController().getLeftX()) * Constants.DriveTrain.MAX_MPS,
                        () ->
                                -modifyAxis(_oi.getDriverController().getRightX())
                                        * Constants.DriveTrain.MAX_ANG_VEL,
                        () -> true));
    }

    public Command getAutonomousCommand() {
        return _autoChooser.getSelected();
    }

    private void setupNamedCommand() {
        // TODO
    }

    public void periodic() {
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

    public VisionSubsystem getVision() {
        return _vision;
    }

    @Override
    public String getLogBase() {
        return "RobotContainer";
    }
}
