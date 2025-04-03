package org.frc5687.robot.commands.algae;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.TunableDouble;

public class AutoNetScore extends OutliersCommand {

    private final DriveSubsystem _drive;
    private final Supplier<Double> _ySupplier;

    private static final TunableDouble b = new TunableDouble("AutoNetScore", "b", 2);
    private static final TunableDouble a = new TunableDouble("AutoNetScore", "a", 2);
    private static final TunableDouble elevatorDist =
            new TunableDouble("AutoNetScore", "elevator raise distance (inches)", 40);

    private static final TunableDouble shootDist =
            new TunableDouble("AutoNetScore", "shoot distance (inches)", 4);

    private static final TunableDouble setpointDist =
            new TunableDouble("AutoNetScore", "setpointDist (inches)", 54);

    public AutoNetScore(RobotContainer _container, Supplier<Double> _ySupplier) {
        this._drive = _container.getDrive();
        this._ySupplier = _ySupplier;
        addRequirements(_drive);
    }

    @Override
    protected void execute(double timestamp) {
        Pose2d currentPose =
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
        double targetX =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? FieldConstants.fieldLength / 2.0 + Units.inchesToMeters(setpointDist.get())
                        : FieldConstants.fieldLength / 2.0
                                - Units.inchesToMeters(setpointDist.get()); // FIXME real value FIXME alliance
        double error = targetX - currentPose.getX();

        Rotation2d targetTheta =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero;
        Rotation2d errorAngle = targetTheta.minus(RobotStateManager.getInstance().getRawIMURotation());

        ChassisSpeeds fieldRelative =
                new ChassisSpeeds(b.get() * error, _ySupplier.get(), a.get() * errorAngle.getRadians());

        _drive.setDesiredChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelative, _drive.getHeading()));
    }

    public static boolean isCloseEnoughToShoot() {
        Pose2d currentPose =
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();

        double targetX =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? FieldConstants.fieldLength / 2.0 + Units.inchesToMeters(setpointDist.get())
                        : FieldConstants.fieldLength / 2.0 - Units.inchesToMeters(setpointDist.get());
        double error = targetX - currentPose.getX();
        Rotation2d targetTheta =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero;
        Rotation2d errorAngle = targetTheta.minus(RobotStateManager.getInstance().getRawIMURotation());
        return Math.abs(error) < Units.inchesToMeters(shootDist.get())
                && Math.abs(errorAngle.getDegrees()) < 15;
    }

    public static boolean isCloseEnoughToRaise() {
        Pose2d currentPose =
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_SWERVE).toPose2d();
        double targetX =
                DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        ? FieldConstants.fieldLength / 2.0 + Units.inchesToMeters(setpointDist.get())
                        : FieldConstants.fieldLength / 2.0 - Units.inchesToMeters(setpointDist.get());
        double error = targetX - currentPose.getX();
        return Math.abs(error) < Units.inchesToMeters(elevatorDist.get());
    }
}
