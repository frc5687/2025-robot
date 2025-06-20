package org.frc5687.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5687.robot.commands.drive.DriveToGroundAlgae;
import org.frc5687.robot.commands.drive.DriveToGroundCoral;
import org.frc5687.robot.commands.drive.TeleopDriveCommand;
import org.frc5687.robot.commands.drive.TeleopDriveWithSnapTo;
import org.frc5687.robot.util.Helpers;
import org.frc5687.robot.util.OutliersController;
import org.frc5687.robot.util.ReefAlignmentHelpers.ReefSide;

public class OperatorInterface {
    private final OutliersController _driverController;
    private final OutliersController _operatorController;

    public OperatorInterface() {
        _driverController = new OutliersController(new CommandPS5Controller(0));
        _operatorController = new OutliersController(new CommandPS5Controller(1));
    }

    public void configureCommandMapping(RobotContainer container) {
        configureDriverControls(container);
        configureOperatorControls(container);
    }

    private void configureDriverControls(RobotContainer container) {
        _driverController
                .y()
                .onTrue(
                        new TeleopDriveWithSnapTo(
                                Degrees.of(180).in(Radians),
                                container.getDrive(),
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftY())
                                                * Constants.DriveTrain.MAX_MPS,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getLeftX())
                                                * Constants.DriveTrain.MAX_MPS,
                                () ->
                                        -RobotContainer.modifyAxis(getDriverController().getRightX())
                                                * Constants.DriveTrain.MAX_ANG_VEL,
                                () -> true));

        _driverController.rightMiddleButton().onTrue(new InstantCommand(container.getDrive()::zeroIMU));
    }

    private void configureOperatorControls(RobotContainer container) {
        // TODO
    }

    public static double modifyAxis(double value) {
        value = Helpers.applyDeadband(value, 0.1);
        value = Math.copySign(value * value, value);

        return value;
    }

    public static Pose2d getProcessorLineupPose() {
        Pose2d processorPose = new Pose2d(6.2, 0.6, Rotation2d.kCW_90deg);
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            processorPose = FlippingUtil.flipFieldPose(processorPose);
        }
        return processorPose;
    }

    public OutliersController getDriverController() {
        return _driverController;
    }

    public OutliersController getOperatorController() {
        return _operatorController;
    }
}
