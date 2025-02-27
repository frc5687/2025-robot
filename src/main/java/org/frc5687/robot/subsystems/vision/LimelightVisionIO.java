package org.frc5687.robot.subsystems.vision;

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

public class LimelightVisionIO implements VisionIO {
    private final Map<String, String> _cameraNames;
    private final Map<String, Transform3d> _cameraPoses;

    public LimelightVisionIO() {
        _cameraNames = new HashMap<>();
        _cameraPoses = new HashMap<>();

        addCamera("North_Camera", "limelight-center", Constants.Vision.ROBOT_TO_NORTH_CAM);
        // addCamera("North_West_Camera", "limelight-northwest", Constants.Vision.ROBOT_TO_NW_CAM);
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
        inputs.cameraObservations.put(logicalName, new ArrayList<>());

        // Set robot orientation for MegaTag2
        // Use robot's current yaw from RobotStateManager
        double robotYaw = RobotStateManager.getInstance().getRawIMURotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(limelightName, robotYaw, 0, 0, 0, 0, 0);

        LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
        if (results == null || !results.valid) {
            return;
        }

        if (results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
            double timestamp = Timer.getFPGATimestamp() - (results.latency_pipeline / 1000.0);

            for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
                AprilTagObservation observation = AprilTagObservation.fromLimelight(target, timestamp);
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

    @Override
    public void writeOutputs(VisionOutputs outputs) {
        for (String limelightName : _cameraNames.values()) {
            LimelightHelpers.setPipelineIndex(limelightName, outputs.pipelineIndex);
        }
    }
}
