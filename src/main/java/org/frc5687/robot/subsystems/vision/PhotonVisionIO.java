package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.frc5687.robot.Constants;
import org.frc5687.robot.util.FieldConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionIO implements VisionIO {

    private static class CameraConfig {
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;

        CameraConfig(String name, Transform3d robotToCamera) {
            this.camera = new PhotonCamera(name);
            this.estimator =
                    new PhotonPoseEstimator(
                            FieldConstants.aprilTagLayout,
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            robotToCamera);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        }
    }

    private final Map<String, CameraConfig> _cameras;

    public PhotonVisionIO() {
        _cameras = new HashMap<>();
        addCamera("North_Camera", Constants.Vision.ROBOT_TO_NORTH_CAM);
        addCamera("North_West_Camera", Constants.Vision.ROBOT_TO_NW_CAM);
    }

    private void addCamera(String cameraName, Transform3d robotToCamera) {
        _cameras.put(cameraName, new CameraConfig(cameraName, robotToCamera));
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.estimatedPoses = new HashMap<>();
        updateCameraInputs(inputs, "North_Camera");
        updateCameraInputs(inputs, "North_West_Camera");
    }

    private void updateCameraInputs(VisionInputs inputs, String cameraName) {
        inputs.cameraObservations.put(cameraName, new ArrayList<>());
        CameraConfig cam = _cameras.get(cameraName);
        List<PhotonPipelineResult> results = cam.camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            PhotonPipelineResult mostRecentResult = results.get(results.size() - 1);
            Optional<EstimatedRobotPose> estimatedPose = cam.estimator.update(mostRecentResult);
            if (estimatedPose.isPresent()) {
                inputs.estimatedPoses.put(cameraName, estimatedPose.get());
            }
            for (PhotonTrackedTarget target : mostRecentResult.targets) {
                inputs
                        .cameraObservations
                        .get(cameraName)
                        .add(
                                AprilTagObservation.fromPhotonVision(
                                        target, mostRecentResult.getTimestampSeconds()));
            }
        }
    }

    @Override
    public void writeOutputs(VisionOutputs outputs) {
        for (var cam : _cameras.values()) {
            cam.camera.setPipelineIndex(outputs.pipelineIndex);
        }
    }
}
