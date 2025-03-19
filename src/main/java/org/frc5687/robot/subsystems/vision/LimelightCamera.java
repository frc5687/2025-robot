package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.util.vision.AprilTagObservation;
import org.frc5687.robot.util.vision.LimelightHelpers;
import org.frc5687.robot.util.vision.LimelightHelpers.LimelightResults;
import org.frc5687.robot.util.vision.LimelightHelpers.LimelightTarget_Fiducial;
import org.frc5687.robot.util.vision.LimelightHelpers.PoseEstimate;
import org.frc5687.robot.util.vision.LimelightHelpers.RawFiducial;
import org.frc5687.robot.util.vision.NeuralPipelineObservation;

public class LimelightCamera extends Camera {

    private final String _limelightName;
    private int currentPipeline = -999; // random number...

    public LimelightCamera(String limelightName) {
        _limelightName = limelightName;
    }

    @Override
    public VisionResult getLatestResult() {
        List<AprilTagObservation> tags = new ArrayList<>();
        List<NeuralPipelineObservation> neuralObservations = new ArrayList<>();
        Optional<RobotPoseEstimate> poseEstimate = Optional.empty();

        LimelightHelpers.SetIMUMode(_limelightName, 0);
        double robotYaw =
                RobotStateManager.getInstance()
                        .getRawIMURotation()
                        .getDegrees(); // TODO use a different mode
        LimelightHelpers.SetRobotOrientation(_limelightName, robotYaw, 0, 0, 0, 0, 0);

        LimelightResults results = LimelightHelpers.getLatestResults(_limelightName);
        if (results == null) {
            System.err.println("Results were null for camera " + _limelightName);
        } else if (!results.valid) {
            // System.err.println("Results were invalid for camera " + _limelightName);
        } else {
            double[] t2d = LimelightHelpers.getT2DArray(_limelightName);
            RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(_limelightName);
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
                        System.err.println(
                                "April tag observation "
                                        + target.fiducialID
                                        + " on camera "
                                        + _limelightName
                                        + " did not have matching raw fiducial");
                        observation = AprilTagObservation.fromLimelight(target, t2d, timestamp);
                    }

                    tags.add(observation);
                }

                PoseEstimate mt2PoseEstimate =
                        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(_limelightName);
                if (LimelightHelpers.validPoseEstimate(mt2PoseEstimate)) {
                    RobotPoseEstimate robotPose =
                            RobotPoseEstimate.fromLimelight(mt2PoseEstimate, _limelightName);
                    poseEstimate = Optional.of(robotPose);
                }
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
        if (index == -1)
            throw new IllegalArgumentException(
                    "Driver mode is only supported on PhotonVision cameras. Failed to set Limelight camera "
                            + _limelightName
                            + " to driver mode.");
        LimelightHelpers.setPipelineIndex(_limelightName, index);
    }

    @Override
    public int getCurrentPipelineIndex() {
        return (int) LimelightHelpers.getCurrentPipelineIndex(_limelightName);
    }
}
