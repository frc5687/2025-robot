package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.util.vision.AprilTagObservation;
import org.frc5687.robot.util.vision.LimelightHelpers;
import org.frc5687.robot.util.vision.LimelightHelpers.LimelightResults;
import org.frc5687.robot.util.vision.LimelightHelpers.LimelightTarget_Fiducial;
import org.frc5687.robot.util.vision.LimelightHelpers.PoseEstimate;
import org.frc5687.robot.util.vision.LimelightHelpers.RawFiducial;

public class LimelightVisionIO implements VisionIO {
    private final Map<String, String> _cameraNames;
    private final Map<String, Transform3d> _cameraPoses;

    public LimelightVisionIO() {
        _cameraNames = new HashMap<>();
        _cameraPoses = new HashMap<>();

        addCamera("North_Camera", "limelight-center", Constants.Vision.ROBOT_TO_NORTH_CAM);
        addCamera("North_West_Camera", "limelight-left", Constants.Vision.ROBOT_TO_NW_CAM);
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

    @Override
    public void resetCameraIMU(Rotation2d heading) {
        // for (String camera : _cameraNames.values()) {
        //     LimelightHelpers.SetIMUMode(camera, 1);
        //     LimelightHelpers.SetRobotOrientation(camera, heading.getDegrees(), 0, 0, 0, 0, 0);
        //     LimelightHelpers.SetIMUMode(camera, 2);
        // }
    }

    private void updateCameraInputs(VisionInputs inputs, String logicalName) {
        String limelightName = _cameraNames.get(logicalName);
        inputs.cameraObservations.put(logicalName, new ArrayList<>());

        // Set robot orientation for MegaTag2
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

                inputs.cameraObservations.get(logicalName).add(observation);
            }

            PoseEstimate poseEstimate =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
            if (LimelightHelpers.validPoseEstimate(poseEstimate)) {
                RobotPoseEstimate robotPose = RobotPoseEstimate.fromLimelight(poseEstimate, logicalName);
                inputs.estimatedPoses.put(logicalName, robotPose);
            }
        }
    }

    // private void updateCameraInputs(VisionInputs inputs, String logicalName) {
    //     String limelightName = _cameraNames.get(logicalName);
    //     inputs.cameraObservations.put(logicalName, new ArrayList<>());

    //     // Set robot orientation for MegaTag2
    //     // Use robot's current yaw from RobotStateManager
    //     double robotYaw = RobotStateManager.getInstance().getRawIMURotation().getDegrees();
    //     LimelightHelpers.SetRobotOrientation(limelightName, robotYaw, 0, 0, 0, 0, 0);

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
        for (String limelightName : _cameraNames.values()) {
            LimelightHelpers.setPipelineIndex(limelightName, outputs.pipelineIndex);
        }
    }
}
