// SimVisionIO.java
package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.*;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

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
    public static final Transform3d ROBOT_TO_CENTER_CAMERA =
            new Transform3d(
                    new Translation3d(0.281, 0.038, 0.217),
                    new Rotation3d(0, Units.degreesToRadians(-15), 0));

    private static final Transform3d ROBOT_TO_RIGHT_CAMERA =
            new Transform3d(
                    new Translation3d(0.281, -0.200, 0.217),
                    new Rotation3d(0, Units.degreesToRadians(-15), 0));

    private static final Transform3d ROBOT_TO_LEFT_CAMERA =
            new Transform3d(
                    new Translation3d(-0.281, 0.200, 0.217),
                    new Rotation3d(0, Units.degreesToRadians(-15), Math.PI));

    public SimVisionIO() {
        _visionSim = new VisionSystemSim("MainVision");

        AprilTagFieldLayout layout = null;
        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
            _visionSim.addAprilTags(layout);
        } catch (Exception e) {
            System.err.println("Failed to load AprilTag layout: " + e.getMessage());
            return;
        }

        addCamera("CenterCamera", ROBOT_TO_CENTER_CAMERA, layout);
        addCamera("RightCamera", ROBOT_TO_RIGHT_CAMERA, layout);
        addCamera("LeftCamera", ROBOT_TO_LEFT_CAMERA, layout);
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
        inputs.visionTimestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        for (var entry : _cameras.entrySet()) {
            String cameraName = entry.getKey();
            CameraConfig config = entry.getValue();

            List<PhotonPipelineResult> results = config.camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                PhotonPipelineResult result = results.get(results.size() - 1);

                if (result.hasTargets()) {
                    AprilTagObservation[] observations = new AprilTagObservation[result.targets.size()];
                    for (int i = 0; i < result.targets.size(); i++) {
                        observations[i] =
                                AprilTagObservation.fromPhotonVision(
                                        result.targets.get(i), result.getTimestampSeconds());
                    }

                    if (cameraName.equals("CenterCamera")) {
                        inputs.centerCameraObservations = Arrays.asList(observations);
                        inputs.hasTargets = true;
                        inputs.numTags = observations.length;
                    }

                    Optional<EstimatedRobotPose> poseEstimate = config.estimator.update(result);
                    if (poseEstimate.isPresent()) {
                        inputs.estimatedPoses.put(cameraName, poseEstimate.get());
                    }
                }
            }
        }

        _visionSim.update(_robotState.getPose(RobotCoordinate.ROBOT_BASE_SIM_ODOM).toPose2d());
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
