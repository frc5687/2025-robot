package org.frc5687.robot.util.estimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.subsystems.drive.DriveInputs;
import org.frc5687.robot.subsystems.vision.RobotPoseEstimate;

public class RobustSwerveOdometry implements EstimatorLoggable {
    private static final double VELOCITY_SAMPLE_TIME = 0.02;
    private static final double MAX_VISION_LATENCY = 0.3;
    private static final double MIN_VELOCITY_FOR_SKID_DETECTION = 0.05; // m/s
    private static final double MAX_POSITION_CHANGE =
            0.1; // Maximum position change per cycle in meters
    private static final double MAX_VISION_JUMP =
            0.2; // Maximum vision correction per cycle in meters
    private static final double POSE_BUFFER_SECONDS = 1.5; // Buffer size for pose history

    private final ModuleState[] _moduleStates;
    private final SkidDetector _skidDetector;
    private final CollisionDetector _collisionDetector;
    private final FOMCalculator _fomCalculator;

    // Pose buffer for handling latency
    private final TimeInterpolatableBuffer<Pose2d> _poseBuffer =
            TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SECONDS);

    private Pose2d _robotPose = new Pose2d();
    private Pose2d _previousRobotPose = new Pose2d();
    private Translation2d _robotVelocity = new Translation2d();
    private double _lastTimestamp = 0.0;
    private boolean _firstCycle = true;
    private double _currentVelocity = 0.0; // Scalar velocity magnitude

    public RobustSwerveOdometry(Translation2d[] moduleLocations) {
        _moduleStates = new ModuleState[moduleLocations.length];
        for (int i = 0; i < moduleLocations.length; i++) {
            _moduleStates[i] = new ModuleState(moduleLocations[i]);
        }

        _skidDetector = new SkidDetector(moduleLocations.length);
        _collisionDetector = new CollisionDetector();
        _fomCalculator = new FOMCalculator();

        _lastTimestamp = Timer.getFPGATimestamp();
    }

    public void update(DriveInputs inputs) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - _lastTimestamp;

        if (_firstCycle) {
            _firstCycle = false;
            _lastTimestamp = currentTime;

            // Initialize pose buffer with current pose
            _poseBuffer.addSample(currentTime, _robotPose);
            return; // Skip first cycle to establish baseline
        }

        _previousRobotPose = _robotPose;

        double[] moduleVelocities = new double[_moduleStates.length];

        for (int i = 0; i < _moduleStates.length; i++) {
            _moduleStates[i].update(
                    inputs.modulePositions[i].distanceMeters,
                    inputs.measuredStates[i].angle,
                    inputs.yawPosition,
                    currentTime,
                    _lastTimestamp);

            moduleVelocities[i] = Math.abs(_moduleStates[i].getVelocity());
        }

        boolean skidding = _skidDetector.detect(moduleVelocities);

        double accelX = inputs.rollVelocityRadPerSec;
        double accelY = inputs.pitchVelocityRadPerSec;
        boolean collision = _collisionDetector.detect(accelX, accelY);

        _fomCalculator.updateOdometryFOM(skidding, collision);

        Translation2d weightedSum = new Translation2d();
        double totalWeight = 0.0;

        double averageVelocity = 0;
        for (double velocity : moduleVelocities) {
            averageVelocity += velocity;
        }
        averageVelocity /= moduleVelocities.length;

        if (skidding && averageVelocity > MIN_VELOCITY_FOR_SKID_DETECTION) {
            log("Skidding", true);

            double minVelocity = Double.MAX_VALUE;
            for (double velocity : moduleVelocities) {
                if (velocity > MIN_VELOCITY_FOR_SKID_DETECTION) {
                    minVelocity = Math.min(minVelocity, velocity);
                }
            }

            if (minVelocity == Double.MAX_VALUE) {
                minVelocity = 0.01; // Fallback if no modules are above threshold
            }

            double skidThreshold = minVelocity * 1.5; // Modules with velocity 50% higher than minimum

            for (int i = 0; i < _moduleStates.length; i++) {
                if (moduleVelocities[i] > skidThreshold) {
                    log("SkippingModule", i);
                    continue;
                }

                Pose2d modulePose = _moduleStates[i].getEstimatedRobotPose();
                double trust = _moduleStates[i].getTrustFactor();

                weightedSum = weightedSum.plus(modulePose.getTranslation().times(trust));
                totalWeight += trust;
            }
        } else {
            for (int i = 0; i < _moduleStates.length; i++) {
                Pose2d modulePose = _moduleStates[i].getEstimatedRobotPose();
                double trust = _moduleStates[i].getTrustFactor();

                weightedSum = weightedSum.plus(modulePose.getTranslation().times(trust));
                totalWeight += trust;
            }
        }

        Translation2d newTranslation;
        if (totalWeight > 0) {
            newTranslation = weightedSum.times(1.0 / totalWeight);

            Translation2d positionDelta = newTranslation.minus(_previousRobotPose.getTranslation());
            double distanceMoved = positionDelta.getNorm();

            if (distanceMoved > MAX_POSITION_CHANGE) {
                log("PositionChangeConstrained", distanceMoved);
                positionDelta = positionDelta.times(MAX_POSITION_CHANGE / distanceMoved);
                newTranslation = _previousRobotPose.getTranslation().plus(positionDelta);
            }
        } else {
            newTranslation = _previousRobotPose.getTranslation();
            log("UsingPreviousPosition", true);
        }

        Pose2d newPose = new Pose2d(newTranslation, inputs.yawPosition);

        if (dt > VELOCITY_SAMPLE_TIME) {
            _robotVelocity = newTranslation.minus(_previousRobotPose.getTranslation()).times(1.0 / dt);
            _currentVelocity = _robotVelocity.getNorm();
        }

        _robotPose = newPose;

        // Add the new pose to our time-interpolatable buffer
        _poseBuffer.addSample(currentTime, _robotPose);

        inputs.estimatedPose = _robotPose;

        _lastTimestamp = currentTime;

        log("X", _robotPose.getX());
        log("Y", _robotPose.getY());
        log("Heading", _robotPose.getRotation().getDegrees());
        log("Velocity", _currentVelocity);
        log("DeltaTime", dt);
        log("Skidding", skidding);
    }

    public void addVisionMeasurement(RobotPoseEstimate poseEstimate) {
        double measurementTimestamp = poseEstimate.timestampSeconds;

        if (Timer.getFPGATimestamp() - measurementTimestamp > MAX_VISION_LATENCY) {
            log("VisionStatus", "Rejected (Stale)");
            return;
        }

        if (_skidDetector.isSkidding()) {
            log("VisionStatus", "Rejected (Skidding)");
            return;
        }

        var maybePose = _poseBuffer.getSample(measurementTimestamp);
        if (maybePose.isEmpty()) {
            log("VisionStatus", "Rejected (No historical pose)");
            return;
        }

        Pose2d poseAtVisionTime = maybePose.get();

        Transform2d odometryToCurrentTransform = new Transform2d(poseAtVisionTime, _robotPose);

        double visionFOM = _fomCalculator.calculateVisionFOM(poseEstimate, poseAtVisionTime);
        double odometryFOM = _fomCalculator.getOdometryFOM();

        if (visionFOM < odometryFOM) {
            double[] weights = FOMCalculator.calculateBlendWeights(visionFOM, odometryFOM);
            double visionWeight = weights[0];
            double odometryWeight = weights[1];

            Translation2d poseDiff =
                    poseEstimate.pose.getTranslation().minus(poseAtVisionTime.getTranslation());
            double poseDiffNorm = poseDiff.getNorm();

            if (poseDiffNorm > MAX_VISION_JUMP) {
                log("VisionJumpConstrained", poseDiffNorm);
                visionWeight *= (MAX_VISION_JUMP / poseDiffNorm);
                odometryWeight = 1.0 - visionWeight;
            }

            Translation2d blendedHistoricalTranslation =
                    poseEstimate
                            .pose
                            .getTranslation()
                            .times(visionWeight)
                            .plus(poseAtVisionTime.getTranslation().times(odometryWeight));

            Rotation2d blendedHistoricalRotation;
            if (poseEstimate.tagCount >= 2) {
                double visionRotationWeight = Math.min(0.8, visionWeight);
                double odometryRotationWeight = 1.0 - visionRotationWeight;

                double visionAngle = poseEstimate.pose.getRotation().getRadians();
                double odometryAngle = poseAtVisionTime.getRotation().getRadians();

                double angleDiff = visionAngle - odometryAngle;
                if (angleDiff > Math.PI) {
                    visionAngle -= 2 * Math.PI;
                } else if (angleDiff < -Math.PI) {
                    visionAngle += 2 * Math.PI;
                }

                double blendedAngle =
                        (visionAngle * visionRotationWeight) + (odometryAngle * odometryRotationWeight);
                blendedHistoricalRotation = new Rotation2d(blendedAngle);
            } else {
                double visionAngle = poseEstimate.pose.getRotation().getRadians();
                double odometryAngle = poseAtVisionTime.getRotation().getRadians();

                double angleDiff = visionAngle - odometryAngle;
                if (angleDiff > Math.PI) {
                    visionAngle -= 2 * Math.PI;
                } else if (angleDiff < -Math.PI) {
                    visionAngle += 2 * Math.PI;
                }

                double visionRotationWeight = Math.min(0.5, visionWeight);
                double odometryRotationWeight = 1.0 - visionRotationWeight;

                double blendedAngle =
                        (visionAngle * visionRotationWeight) + (odometryAngle * odometryRotationWeight);
                blendedHistoricalRotation = new Rotation2d(blendedAngle);
            }

            Pose2d blendedHistoricalPose =
                    new Pose2d(blendedHistoricalTranslation, blendedHistoricalRotation);

            _robotPose = blendedHistoricalPose.plus(odometryToCurrentTransform);

            log("VisionStatus", "Applied (Better FOM)");
            log("VisionWeight", visionWeight);
            log("OdometryWeight", odometryWeight);
            log("VisionTimestamp", measurementTimestamp);
        } else {
            log("VisionStatus", "Rejected (Worse FOM)");
        }
    }

    public void resetPose(Pose2d pose) {
        _previousRobotPose = pose;
        _robotPose = pose;
        _robotVelocity = new Translation2d();
        _currentVelocity = 0.0;
        _firstCycle = true;

        for (ModuleState state : _moduleStates) {
            state.reset(pose);
        }

        _skidDetector.reset();
        _collisionDetector.reset();
        _fomCalculator.reset();

        // Clear and initialize the pose buffer
        _poseBuffer.clear();
        _poseBuffer.addSample(Timer.getFPGATimestamp(), pose);

        _lastTimestamp = Timer.getFPGATimestamp();
    }

    public Pose2d getPose() {
        return _robotPose;
    }

    public Translation2d getVelocity() {
        return _robotVelocity;
    }

    public double getScalarVelocity() {
        return _currentVelocity;
    }

    public double getFOM() {
        return _fomCalculator.getRobotFOM();
    }

    public boolean isPoseReliable() {
        return _fomCalculator.isPoseReliable();
    }

    public boolean isSkidding() {
        return _skidDetector.isSkidding();
    }

    public boolean isInCollisionState() {
        return _collisionDetector.isInCollisionState();
    }

    /**
     * Get a pose from the history buffer at a specific timestamp
     *
     * @param timestamp The timestamp to retrieve the pose for
     * @return The pose at the specified time, if available
     */
    public java.util.Optional<Pose2d> getPoseAtTime(double timestamp) {
        return _poseBuffer.getSample(timestamp);
    }
}
