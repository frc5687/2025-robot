package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.frc5687.robot.util.BaseInputs;
import org.photonvision.EstimatedRobotPose;

@Logged
public class VisionInputs extends BaseInputs {
    @NotLogged public Map<String, List<AprilTagObservation>> cameraObservations = new HashMap<>();
    @NotLogged public Map<String, EstimatedRobotPose> estimatedPoses = new HashMap<>();
}
