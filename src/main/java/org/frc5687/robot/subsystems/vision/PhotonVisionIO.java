package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.vision.AprilTagObservation;
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
            estimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
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
        inputs.estimatedPoses.clear();
        updateCameraInputs(inputs, "North_Camera");
        updateCameraInputs(inputs, "North_West_Camera");
    }

    private void updateCameraInputs(VisionInputs inputs, String cameraName) {
        inputs.cameraAprilTagObservations.put(cameraName, new ArrayList<>());
        CameraConfig cam = _cameras.get(cameraName);
        List<PhotonPipelineResult> results = cam.camera.getAllUnreadResults();

        cam.estimator.addHeadingData(
                Timer.getFPGATimestamp(), RobotStateManager.getInstance().getRawIMURotation());

        if (!results.isEmpty()) {
            PhotonPipelineResult mostRecentResult = results.get(results.size() - 1);

            for (PhotonTrackedTarget target : mostRecentResult.targets) {
                AprilTagObservation observation =
                        AprilTagObservation.fromPhotonVision(target, mostRecentResult.getTimestampSeconds());
                inputs.cameraAprilTagObservations.get(cameraName).add(observation);
            }

            Optional<EstimatedRobotPose> photonPoseEstimate = cam.estimator.update(mostRecentResult);

            photonPoseEstimate.ifPresent(
                    pose -> {
                        RobotPoseEstimate robotPose = RobotPoseEstimate.fromPhotonVision(pose, cameraName);
                        inputs.estimatedPoses.put(cameraName, robotPose);
                    });
        }
    }

    @Override
    public void writeOutputs(VisionOutputs outputs) {
        for (Entry<String, Integer> targetPipeline : outputs.targetPipelines.entrySet()) {
            if (_cameras.containsKey(targetPipeline.getKey())) {
                _cameras.get(targetPipeline.getKey()).camera.setPipelineIndex(targetPipeline.getValue());
            } else {
                System.err.println("Invalid camera " + targetPipeline.getKey());
            }
        }
    }
}
