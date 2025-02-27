package org.frc5687.robot.util.estimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.subsystems.drive.DriveInputs;
import org.frc5687.robot.subsystems.vision.RobotPoseEstimate;

public class PoseTracker implements EstimatorLoggable {
    private final DriveInputs _inputs;
    private final RobustSwerveOdometry _robustOdometry;

    private static final double VISION_PROCESSING_INTERVAL = 0.05; // seconds

    private double _lastVisionProcessTime = 0;

    public PoseTracker(Translation2d[] moduleLocations, DriveInputs inputs) {

        _inputs = inputs;
        _robustOdometry = new RobustSwerveOdometry(moduleLocations);
    }

    public void update() {
        _robustOdometry.update(_inputs);
    }

    public void processVision(RobotPoseEstimate poseEstimate, boolean forceProcess) {
        double currentTime = Timer.getFPGATimestamp();

        if (!forceProcess && (currentTime - _lastVisionProcessTime < VISION_PROCESSING_INTERVAL)) {
            return;
        }

        _robustOdometry.addVisionMeasurement(poseEstimate);
        _lastVisionProcessTime = currentTime;
    }

    public void processVision(RobotPoseEstimate poseEstimate) {
        processVision(poseEstimate, false);
    }

    public void resetPose(Pose2d pose) {
        _robustOdometry.resetPose(pose);
        _inputs.estimatedPose = pose;
    }

    public Pose2d getPose() {
        return _inputs.estimatedPose;
    }

    public Translation2d getVelocity() {
        return _robustOdometry.getVelocity();
    }

    public double getFOM() {
        return _robustOdometry.getFOM();
    }

    public boolean isPoseReliable() {
        return _robustOdometry.isPoseReliable();
    }

    public boolean isInCollisionState() {
        return _robustOdometry.isInCollisionState();
    }

    public boolean isSkidding() {
        return _robustOdometry.isSkidding();
    }
}
