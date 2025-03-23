package org.frc5687.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;

public class COMVelocityLimiter {
    private final RobotStateManager _stateManager;

    private final TunableDouble _height1 = new TunableDouble("COMLimit", "Height1", 0.3);
    private final TunableDouble _speedFactor1 = new TunableDouble("COMLimit", "SpeedFactor1", 1.0);

    private final TunableDouble _height2 = new TunableDouble("COMLimit", "Height2", 0.7);
    private final TunableDouble _speedFactor2 = new TunableDouble("COMLimit", "SpeedFactor2", 0.9);

    private final TunableDouble _height3 = new TunableDouble("COMLimit", "Height3", 1.0);
    private final TunableDouble _speedFactor3 = new TunableDouble("COMLimit", "SpeedFactor3", 0.7);

    private final TunableDouble _height4 =
            new TunableDouble(
                    "COMLimit", "Height4", 1.5); // these were based on final 3d pose height, maybe change.
    private final TunableDouble _speedFactor4 = new TunableDouble("COMLimit", "SpeedFactor4", 0.4);

    private double _lastSpeedFactor = 1.0;
    private double _elevatorHeight = 0.0;

    public COMVelocityLimiter() {
        _stateManager = RobotStateManager.getInstance();
    }

    public double getElevatorHeight() {
        Pose3d elevatorTop = _stateManager.getPose(RobotCoordinate.ELEVATOR_TOP);
        if (elevatorTop != null) {
            _elevatorHeight = elevatorTop.getZ();
            return _elevatorHeight;
        }
        return 0.0;
    }

    public double calculateSpeedFactor() {
        double elevatorHeight = getElevatorHeight();

        double h1 = _height1.get();
        double h2 = _height2.get();
        double h3 = _height3.get();
        double h4 = _height4.get();

        double s1 = _speedFactor1.get();
        double s2 = _speedFactor2.get();
        double s3 = _speedFactor3.get();
        double s4 = _speedFactor4.get();

        double speedFactor;

        // LOL
        if (elevatorHeight <= h1) {
            speedFactor = s1;
        } else if (elevatorHeight <= h2) {
            speedFactor = lerp(s1, s2, (elevatorHeight - h1) / (h2 - h1));
        } else if (elevatorHeight <= h3) {
            speedFactor = lerp(s2, s3, (elevatorHeight - h2) / (h3 - h2));
        } else if (elevatorHeight <= h4) {
            speedFactor = lerp(s3, s4, (elevatorHeight - h3) / (h4 - h3));
        } else {
            speedFactor = s4;
        }

        _lastSpeedFactor = MathUtil.clamp(speedFactor, 0.1, 1.0);
        return _lastSpeedFactor;
    }

    private double lerp(double start, double end, double t) {
        t = MathUtil.clamp(t, 0.0, 1.0);
        return start + t * (end - start);
    }

    public ChassisSpeeds limitSpeeds(ChassisSpeeds requestedSpeeds) {
        double speedFactor = calculateSpeedFactor();

        double maxSpeed = Constants.DriveTrain.MAX_MPS;

        double requestedMagnitude =
                Math.hypot(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond);

        double speedLimit = maxSpeed * speedFactor;

        if (requestedMagnitude > speedLimit && requestedMagnitude > 0.01) {
            double scaleFactor = speedLimit / requestedMagnitude;

            return new ChassisSpeeds(
                    requestedSpeeds.vxMetersPerSecond * scaleFactor,
                    requestedSpeeds.vyMetersPerSecond * scaleFactor,
                    requestedSpeeds.omegaRadiansPerSecond * scaleFactor);
        }

        return requestedSpeeds;
    }

    public double getLastSpeedFactor() {
        return _lastSpeedFactor;
    }

    public double getLastElevatorHeight() {
        return _elevatorHeight;
    }
}
