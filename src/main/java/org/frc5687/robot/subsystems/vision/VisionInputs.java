package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.frc5687.robot.util.BaseInputs;
import org.frc5687.robot.util.vision.AprilTagObservation;

public class VisionInputs extends BaseInputs {
    @Logged double test = 0;
    @NotLogged public Map<String, List<AprilTagObservation>> cameraObservations = new HashMap<>();

    // Using RobotPoseEstimate instead of PhotonVision's EstimatedRobotPose
    @NotLogged public Map<String, RobotPoseEstimate> estimatedPoses = new HashMap<>();
}
