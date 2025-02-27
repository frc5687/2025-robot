package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.vision.AprilTagObservation;
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

        addCamera("North_Camera", Constants.Vision.ROBOT_TO_NORTH_CAM, layout);
        addCamera("North_West_Camera", Constants.Vision.ROBOT_TO_NW_CAM, layout);
    }

    private void addCamera(String name, Transform3d robotToCamera, AprilTagFieldLayout layout) {
        PhotonCamera camera = new PhotonCamera(name);

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(88));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(60);
        cameraProp.setAvgLatencyMs(20);
        cameraProp.setLatencyStdDevMs(5);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableRawStream(true);
        cameraSim.setMaxSightRange(4.0);

        PhotonPoseEstimator estimator =
                new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);

        _visionSim.addCamera(cameraSim, robotToCamera);
        _cameras.put(name, new CameraConfig(name, robotToCamera, camera, cameraSim, estimator));
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.estimatedPoses.clear();
        updateCameraInputs(inputs, "North_Camera");
        updateCameraInputs(inputs, "North_West_Camera");
        _visionSim.update(_robotState.getPose(RobotCoordinate.ROBOT_BASE_SIM_ODOM).toPose2d());
    }

    private void updateCameraInputs(VisionInputs inputs, String cameraName) {
        inputs.cameraObservations.put(cameraName, new ArrayList<>());
        CameraConfig cam = _cameras.get(cameraName);
        List<PhotonPipelineResult> results = cam.camera.getAllUnreadResults();
        cam.estimator.addHeadingData(Timer.getFPGATimestamp(), _robotState.getRawIMURotation());

        if (!results.isEmpty()) {
            PhotonPipelineResult mostRecentResult = results.get(results.size() - 1);
            Optional<EstimatedRobotPose> photonPoseEstimate = cam.estimator.update(mostRecentResult);

            for (PhotonTrackedTarget target : mostRecentResult.targets) {
                AprilTagObservation observation =
                        AprilTagObservation.fromPhotonVision(target, mostRecentResult.getTimestampSeconds());
                inputs.cameraObservations.get(cameraName).add(observation);
            }

            photonPoseEstimate.ifPresent(
                    pose -> {
                        boolean usedMultitag = mostRecentResult.multitagResult.isPresent();

                        if (usedMultitag || mostRecentResult.targets.size() > 0) {
                            RobotPoseEstimate robotPose =
                                    RobotPoseEstimate.fromPhotonVision(pose, mostRecentResult, cameraName);
                            inputs.estimatedPoses.put(cameraName, robotPose);
                        }
                    });
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
