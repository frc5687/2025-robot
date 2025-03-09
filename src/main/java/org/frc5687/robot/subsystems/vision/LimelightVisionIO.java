package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.util.EpilogueLog;
import org.frc5687.robot.util.vision.AprilTagObservation;
import org.frc5687.robot.util.vision.LimelightHelpers;
import org.frc5687.robot.util.vision.LimelightHelpers.LimelightResults;
import org.frc5687.robot.util.vision.LimelightHelpers.LimelightTarget_Fiducial;
import org.frc5687.robot.util.vision.LimelightHelpers.PoseEstimate;
import org.frc5687.robot.util.vision.LimelightHelpers.RawDetection;
import org.frc5687.robot.util.vision.LimelightHelpers.RawFiducial;
import org.frc5687.robot.util.vision.NeuralPipelineObservation;

public class LimelightVisionIO implements VisionIO, EpilogueLog {
    private final Map<String, String> _cameraNames;
    private final Map<String, Transform3d> _cameraPoses;

    // private int timer;

    public LimelightVisionIO() {
        _cameraNames = new HashMap<>();
        _cameraPoses = new HashMap<>();

        addCamera("North_Camera", "limelight-center", Constants.Vision.ROBOT_TO_NORTH_CAM);
        addCamera("North_West_Camera", "limelight-left", Constants.Vision.ROBOT_TO_NW_CAM);
        // timer = 0;
    }

    private void addCamera(String logicalName, String limelightName, Transform3d robotToCamera) {
        _cameraNames.put(logicalName, limelightName);
        _cameraPoses.put(logicalName, robotToCamera);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.estimatedPoses.clear();
        for (String logicalName : _cameraNames.keySet()) {
            updateCameraInputs(inputs, logicalName);
        }
    }

    private void updateCameraInputs(VisionInputs inputs, String logicalName) {
        String limelightName = _cameraNames.get(logicalName);
        if (inputs.cameraAprilTagObservations.containsKey(logicalName))
            inputs.cameraAprilTagObservations.get(logicalName).clear();
        else inputs.cameraAprilTagObservations.put(logicalName, new ArrayList<>());
        if (inputs.cameraNeuralPipelineObservations.containsKey(logicalName))
            inputs.cameraNeuralPipelineObservations.get(logicalName).clear();
        else inputs.cameraNeuralPipelineObservations.put(logicalName, new ArrayList<>());

        LimelightHelpers.SetIMUMode(limelightName, 0);
        double robotYaw = RobotStateManager.getInstance().getRawIMURotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(limelightName, robotYaw, 0, 0, 0, 0, 0);

        // Get latest results
        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        if (results == null || !results.valid) {
            return;
        }

        // Get T2D array for target geometry information
        double[] t2d = LimelightHelpers.getT2DArray(limelightName);
        RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightName);

        if (results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
            double timestamp = Timer.getFPGATimestamp() - (results.latency_pipeline / 1000.0);

            for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
                RawFiducial matchingRawFiducial = null;
                for (RawFiducial rawFiducial : rawFiducials) {
                    if (rawFiducial.id == target.fiducialID) {
                        matchingRawFiducial = rawFiducial;
                        break;
                    }
                }

                AprilTagObservation observation;
                if (matchingRawFiducial != null) {
                    observation =
                            AprilTagObservation.fromLimelight(target, matchingRawFiducial, t2d, timestamp);
                } else {
                    observation = AprilTagObservation.fromLimelight(target, t2d, timestamp);
                }

                inputs.cameraAprilTagObservations.get(logicalName).add(observation);
            }

            PoseEstimate poseEstimate =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            if (LimelightHelpers.validPoseEstimate(poseEstimate)) {
                RobotPoseEstimate robotPose = RobotPoseEstimate.fromLimelight(poseEstimate, logicalName);
                inputs.estimatedPoses.put(logicalName, robotPose);
            }
        }

        Transform3d robotToLens = _cameraPoses.get(logicalName);

        RawDetection[] rawDetections = LimelightHelpers.getRawDetections(limelightName);
        // System.out.println(logicalName + " had " + rawDetections.length + " detections");
        for (RawDetection detection : rawDetections) {
            var obs = NeuralPipelineObservation.fromLimelight(detection, robotToLens);
            if (obs == null) continue;
            inputs.cameraNeuralPipelineObservations.get(logicalName).add(obs);
            log("angle (deg)", +new Rotation2d(obs.getX(), obs.getY()).getDegrees());
            log("distance from robot center", Math.hypot(obs.getX(), obs.getY()));
            log("x", obs.getX());
            log("y", obs.getY());
        }
    }

    // private void updateCameraInputs(VisionInputs inputs, String logicalName) {
    //     String limelightName = _cameraNames.get(logicalName);
    //     inputs.cameraObservations.put(logicalName, new ArrayList<>());

    //     // Set robot orientation for MegaTag2
    //     // Use robot's current yaw from RobotStateManager
    //     double robotYaw = RobotStateManager.getInstance().getRawIMURotation().getDegrees();

    //     // This needs you to send the json over NT
    //     LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    //     if (results == null || !results.valid) {
    //         return;
    //     }

    //     RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightName);
    //     // System.out.println(results.targets_Fiducials);
    //     if (results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
    //         double timestamp = Timer.getFPGATimestamp() - (results.latency_pipeline / 1000.0);

    //         for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
    //             AprilTagObservation observation = AprilTagObservation.fromLimelight(target,
    // timestamp);
    //             for (RawFiducial rawFiducial : rawFiducials) {
    //                 if (rawFiducial.id == target.fiducialID) {
    //                     observation = AprilTagObservation.fromLimelight(target, rawFiducial,
    // timestamp);
    //                 }
    //             }
    //             inputs.cameraObservations.get(logicalName).add(observation);
    //         }

    //         PoseEstimate poseEstimate =
    //                 LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    //         if (LimelightHelpers.validPoseEstimate(poseEstimate)) {
    //             RobotPoseEstimate robotPose = RobotPoseEstimate.fromLimelight(poseEstimate,
    // logicalName);
    //             inputs.estimatedPoses.put(logicalName, robotPose);
    //         }
    //     }
    // }

    @Override
    public void writeOutputs(VisionOutputs outputs) {
        for (Entry<String, Integer> targetPipeline : outputs.targetPipelines.entrySet()) {
            if (_cameraNames.containsKey(targetPipeline.getKey())) {
                String limelightName = _cameraNames.get(targetPipeline.getKey());
                LimelightHelpers.setPipelineIndex(limelightName, targetPipeline.getValue());
            } else {
                System.err.println("Invalid camera " + targetPipeline.getKey());
            }
        }
    }

    @Override
    public String getLogBase() {
        return "LimelightVisionIO";
    }
}
