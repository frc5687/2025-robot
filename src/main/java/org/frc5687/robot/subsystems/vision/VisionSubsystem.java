package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.*;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class VisionSubsystem extends OutliersSubsystem<VisionInputs, VisionOutputs> {
    private static final double MIN_AMBIGUITY = 0.3;
    private static final double MAX_LATENCY_MS = 100.0;
    private static final double MULTI_TAG_AMBIGUITY_THRESHOLD = 0.1;

    public VisionSubsystem(RobotContainer container, VisionIO io) {
        super(container, io, new VisionInputs(), new VisionOutputs());
    }

    public void setPipeline(int index) {
        _outputs.pipelineIndex = index;
    }

    public boolean hasValidTag(AprilTagObservation observation) {
        if (observation == null) return false;

        double ambiguityThreshold = MIN_AMBIGUITY;
        double latencyMs = observation.getLatency() * 1000.0;

        return observation.getAmbiguity() < ambiguityThreshold && latencyMs < MAX_LATENCY_MS;
    }

    public AprilTagObservation[] getValidTags() {
        List<AprilTagObservation> res = new ArrayList<>();
        for (List<AprilTagObservation> observations : _inputs.cameraObservations.values()) {
            res.addAll(observations);
        }
        return (AprilTagObservation[]) res.toArray();
    }

    @Override
    protected void periodic(VisionInputs inputs, VisionOutputs outputs) {
        List<Pose3d> estimatedPoses = new ArrayList<>();
        for (var robotPose : inputs.estimatedPoses.values()) {
            estimatedPoses.add(robotPose.estimatedPose);
            RobotStateManager.getInstance().updateVision(robotPose);
        }
        List<AprilTagObservation> northEastTags =
                inputs.cameraObservations.getOrDefault("North_East_Camera", new ArrayList<>());
        List<AprilTagObservation> northWestTags =
                inputs.cameraObservations.getOrDefault("North_West_Camera", new ArrayList<>());

        log("VisionPoses", estimatedPoses, Pose3d.struct);
        log("NorthEastTags", northEastTags, AprilTagObservation.struct);
        log("NorthWestTags", northWestTags, AprilTagObservation.struct);
    }

    public boolean hasValidTargets() {
        return getValidTags().length > 0;
    }

    public AprilTagObservation getClosestTag() {
        AprilTagObservation[] validTags = getValidTags();
        if (validTags.length == 0) return null;

        // Area of the tag was the best indicator for distance than calculating the distance with the
        // transform. TODO: test best outside of sim
        return Arrays.stream(validTags)
                .max((a, b) -> Double.compare(a.getArea(), b.getArea()))
                .orElse(null);
    }

    public AprilTagObservation getBestTag() {
        AprilTagObservation[] validTags = getValidTags();
        if (validTags.length == 0) return null;

        return Arrays.stream(validTags)
                .min((a, b) -> Double.compare(a.getAmbiguity(), b.getAmbiguity()))
                .orElse(null);
    }

    @Override
    protected void processInputs() {}
}
