package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import java.util.*;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class VisionSubsystem extends OutliersSubsystem<VisionInputs, VisionOutputs> {
    private static final double MIN_AMBIGUITY = 0.3;
    private static final double MAX_LATENCY_MS = 100.0;
    private static final double MULTI_TAG_AMBIGUITY_THRESHOLD = 0.1;
    private static final double MAX_DISTANCE = 4;

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

        return observation.getAmbiguity() < ambiguityThreshold
                && latencyMs < MAX_LATENCY_MS
                && observation.get2dDistance() < MAX_DISTANCE;
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

        log("VisionPoses", estimatedPoses, Pose3d.struct);
        log("NorthTags", getNorthCameraObservations(), AprilTagObservation.struct);
        log("NorthWestTags", getNorthWestCameraObservations(), AprilTagObservation.struct);
    }

    public boolean hasValidTargets() {
        return getValidTags().length > 0;
    }

    public List<AprilTagObservation> getNorthCameraObservations() {
        return _inputs.cameraObservations.getOrDefault("North_Camera", new ArrayList<>());
    }

    public List<AprilTagObservation> getNorthWestCameraObservations() {
        return _inputs.cameraObservations.getOrDefault("North_West_Camera", new ArrayList<>());
    }

    public Optional<AprilTagObservation> getTagFromObservations(
            List<AprilTagObservation> observations, int tagId) {
        return Arrays.stream(observations.toArray(new AprilTagObservation[0]))
                .filter(tag -> (tag.getId() == tagId))
                .findFirst();
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

    // use camera for calibration info out of sim
    public double calculateLateralOffset(AprilTagObservation observation, String cameraName) {
        Translation2d[] corners = observation.getCorners();
        
        double centerX = 0;
        for (Translation2d corner : corners) {
            centerX += corner.getX();
        }
        centerX /= corners.length;
        
        Matrix<N3, N3> calibrationMatrix = Constants.Vision.simCalibrationMatrix;
        double principalX = calibrationMatrix.get(0, 2);
        
        return (centerX - principalX) / principalX;
    }

    public double calculateDistanceWithCalibration(
            AprilTagObservation observation, String cameraName) {

        Matrix<N3, N3> calibrationMatrix = Constants.Vision.simCalibrationMatrix; // TODO FIX
        Translation2d[] corners = observation.getCorners();
        
        // focal length from calibration matrix
        double fx = calibrationMatrix.get(0, 0);
        double fy = calibrationMatrix.get(1, 1);
        double focalLength = (fx + fy) / 2.0; // TODO, if the difference between fx and fy is large, might be better to use geometric mean
        
        // From PhotonVision corners
        double pixelWidth1 = corners[1].getDistance(corners[0]); // Bottom 
        double pixelWidth2 = corners[3].getDistance(corners[2]); // Top
        double pixelHeight1 = corners[0].getDistance(corners[3]); // Left
        double pixelHeight2 = corners[2].getDistance(corners[1]); // Right
        
        double avgPixelSize = (pixelWidth1 + pixelWidth2 + pixelHeight1 + pixelHeight2) / 4.0;
        
        // we can use similar triangels to calculate the distance using the tag size and focal length
        // distance = (actual_size * focal_length) / pixel_size
        return Units.inchesToMeters(6.5) * focalLength / avgPixelSize;
    }

    @Override
    protected void processInputs() {}

    public void setPipelineIndex(int index) {
        _outputs.pipelineIndex = index;
    }
}
