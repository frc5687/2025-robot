package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.vision.AprilTagObservation;
import org.frc5687.robot.util.vision.NeuralPipelineObservation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionCamera extends Camera {

    protected final String _cameraName;
    protected final PhotonCamera _cam;
    protected final PhotonPoseEstimator _estimator;
    protected final Transform3d
            _robotToCam; // will be used to triangulate the neural detections once I do that
    protected int currentPipeline = -999; // random number...

    public PhotonVisionCamera(String cameraName, Transform3d robotToCamera) {
        _cameraName = cameraName;
        _cam = new PhotonCamera(cameraName);
        _estimator =
                new PhotonPoseEstimator(
                        FieldConstants.aprilTagLayout,
                        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                        robotToCamera);
        _estimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
        _robotToCam = robotToCamera;
    }

    @Override
    public VisionResult getLatestResult() {
        List<AprilTagObservation> tags = new ArrayList<>();
        List<NeuralPipelineObservation> neuralObservations = new ArrayList<>();
        Optional<RobotPoseEstimate> poseEstimate = Optional.empty();

        _estimator.addHeadingData(
                Timer.getFPGATimestamp(), RobotStateManager.getInstance().getRawIMURotation());
        List<PhotonPipelineResult> results = _cam.getAllUnreadResults();

        if (!results.isEmpty()) {
            PhotonPipelineResult mostRecentResult = results.get(results.size() - 1);
            for (PhotonTrackedTarget target : mostRecentResult.targets) {
                if (target.objDetectId == -1) {
                    // apriltag detection
                    AprilTagObservation observation =
                            AprilTagObservation.fromPhotonVision(target, mostRecentResult.getTimestampSeconds());
                    tags.add(observation);
                } else {
                    // neural detection
                    var observation = NeuralPipelineObservation.fromPhotonVision(_cam, _robotToCam, target);
                    if (observation.isPresent()) {
                        neuralObservations.add(observation.get());
                    }
                }
            }

            Optional<EstimatedRobotPose> pose = _estimator.update(mostRecentResult);
            if (pose.isPresent()) {
                poseEstimate = Optional.of(RobotPoseEstimate.fromPhotonVision(pose.get(), _cameraName));
            }
        }

        return new VisionResult(tags, neuralObservations, poseEstimate);
    }

    @Override
    public void setPipelineIndex(int index) {
        if (currentPipeline == index) {
            return;
        }
        currentPipeline = index;

        if (index == -1) {
            _cam.setDriverMode(true);
        } else {
            _cam.setDriverMode(false);
            _cam.setPipelineIndex(index);
        }
    }

    @Override
    public int getCurrentPipelineIndex() {
        return _cam.getPipelineIndex();
    }
}
