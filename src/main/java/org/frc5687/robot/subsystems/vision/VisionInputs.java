package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.frc5687.robot.util.BaseInputs;
import org.frc5687.robot.util.vision.AprilTagObservation;
import org.frc5687.robot.util.vision.NeuralPipelineObservation;

public class VisionInputs extends BaseInputs {
    @Logged
    public boolean hack = true; // has to have something logged if there are fields it looks like ..?

    @NotLogged
    public Map<String, List<AprilTagObservation>> cameraAprilTagObservations = new HashMap<>();

    @NotLogged
    public Map<String, List<NeuralPipelineObservation>> cameraNeuralPipelineObservations =
            new HashMap<>();

    // Using RobotPoseEstimate instead of PhotonVision's EstimatedRobotPose
    @NotLogged public Map<String, RobotPoseEstimate> estimatedPoses = new HashMap<>();
}
