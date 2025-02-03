package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.frc5687.robot.util.BaseInputs;
import org.photonvision.EstimatedRobotPose;

@Logged
public class VisionInputs extends BaseInputs {
    @Logged(name = "Vision Timestamp")
    public double visionTimestamp = 0.0;

    @Logged(name = "Has Targets")
    public boolean hasTargets = false;

    @Logged(name = "Number of Tags")
    public int numTags = 0;

    @Logged(name = "Tag Observations")
    public List<AprilTagObservation> centerCameraObservations = new ArrayList<>();

    @NotLogged public Map<String, EstimatedRobotPose> estimatedPoses = new HashMap<>();
}
