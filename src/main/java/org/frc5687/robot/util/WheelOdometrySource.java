package org.frc5687.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.util.function.Supplier;
import org.frc5687.robot.Constants;

public class WheelOdometrySource extends OdometrySource {
    private Supplier<SwerveModulePosition[]> _positionSupplier;
    private Supplier<Rotation2d> _headingSupplier;
    private Pose2d _odometryPose;
    private SwerveModulePosition[] _lastWheelPositions = new SwerveModulePosition[4];

    public WheelOdometrySource(
            Supplier<SwerveModulePosition[]> positionSupplier, Supplier<Rotation2d> headingSupplier) {
        for (int i = 0; i < 4; i++) {
            _lastWheelPositions[i] = new SwerveModulePosition();
        }
        _odometryPose = new Pose2d();
        _positionSupplier = positionSupplier;
        _headingSupplier = headingSupplier;
    }

    @Override
    public Pose2d getOdometryPose() {
        SwerveModulePosition[] wheelPositions = _positionSupplier.get();
        Twist2d twist = Constants.DriveTrain.KINEMATICS.toTwist2d(_lastWheelPositions, wheelPositions);
        _odometryPose = _odometryPose.exp(twist);

        for (int i = 0; i < wheelPositions.length; i++) {
            _lastWheelPositions[i] =
                    new SwerveModulePosition(
                            wheelPositions[i].distanceMeters,
                            new Rotation2d(wheelPositions[i].angle.getRadians()));
        }

        Rotation2d gyroAngle = _headingSupplier.get();
        if (gyroAngle != null) {
            Rotation2d robotAngle = gyroAngle; // todo gyro offset :/
            _odometryPose = new Pose2d(_odometryPose.getTranslation(), robotAngle);
        }

        return _odometryPose;
    }
}
