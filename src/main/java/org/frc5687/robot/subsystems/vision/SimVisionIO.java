// SimVisionIO.java
package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.*;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.util.FieldConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SimVisionIO implements VisionIO {
    private static class CameraConfig {
        final PhotonCamera camera;
        final PhotonCameraSim cameraSim;
        final PhotonPoseEstimator estimator;
        final Transform3d robotToCamera;

        CameraConfig(
                String name,
                Transform3d robotToCamera,
                PhotonCamera camera,
                PhotonCameraSim cameraSim,
                PhotonPoseEstimator estimator) {
            this.camera = camera;
            this.cameraSim = cameraSim;
            this.estimator = estimator;
            this.robotToCamera = robotToCamera;
        }
    }

    private final VisionSystemSim _visionSim;
    private final Map<String, CameraConfig> _cameras = new HashMap<>();
    private final RobotStateManager _robotState = RobotStateManager.getInstance();

    // Not real for robot yet, just was testing.
    public static final Transform3d ROBOT_TO_NE_CAMERA =
            new Transform3d(
                    new Translation3d(0.281, 0.038, 0.217),
                    new Rotation3d(0, Units.degreesToRadians(-15), 0));

    private static final Transform3d ROBOT_TO_NW_CAMERA =
            new Transform3d(
                    new Translation3d(0.281, -0.200, 0.217),
                    new Rotation3d(0, Units.degreesToRadians(-15), 0));

    public SimVisionIO() {
        _visionSim = new VisionSystemSim("MainVision");

        AprilTagFieldLayout layout = null;
        try {
            layout = FieldConstants.aprilTagLayout;
            _visionSim.addAprilTags(layout);
        } catch (Exception e) {
            System.err.println("Failed to load AprilTag layout: " + e.getMessage());
            return;
        }

        addCamera("North_East_Camera", ROBOT_TO_NE_CAMERA, layout);
        addCamera("North_West_Camera", ROBOT_TO_NW_CAMERA, layout);
    }

    private void addCamera(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
        PhotonCamera camera = new PhotonCamera(name);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(81));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(40);
        cameraProp.setAvgLatencyMs(25);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(false);
        cameraSim.enableProcessedStream(false);
        cameraSim.enableRawStream(false);
        cameraSim.setMaxSightRange(1.0);

        PhotonPoseEstimator estimator =
                new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);

        _visionSim.addCamera(cameraSim, robotToCamera);
        _cameras.put(name, new CameraConfig(name, robotToCamera, camera, cameraSim, estimator));
    }

    // TODO: make a queue for time as well and drop old tags
    @Override
    public void updateInputs(VisionInputs inputs) {
        updateCameraInputs(inputs, "North_East_Camera");
        updateCameraInputs(inputs, "North_West_Camera");
        _visionSim.update(_robotState.getPose(RobotCoordinate.ROBOT_BASE_SIM_ODOM).toPose2d());
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
        for (CameraConfig config : _cameras.values()) {
            config.camera.setPipelineIndex(outputs.pipelineIndex);
        }
    }

    public void resetSimulation(Pose2d pose) {
        _visionSim.resetRobotPose(pose);
    }
}
