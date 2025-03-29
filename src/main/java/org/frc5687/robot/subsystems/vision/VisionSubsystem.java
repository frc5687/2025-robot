package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.*;
import java.util.stream.Collectors;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.util.FieldConstants;
import org.frc5687.robot.util.vision.AlgaeTracker;
import org.frc5687.robot.util.vision.AprilTagObservation;
import org.frc5687.robot.util.vision.CoralTracker;
import org.frc5687.robot.util.vision.NeuralPipelineObservation;
import org.frc5687.robot.util.vision.TargetCorners;

public class VisionSubsystem extends OutliersSubsystem<VisionInputs, VisionOutputs> {
    private static final double MAX_LATENCY_MS = 100.0;
    private static final double MAX_AMBIGUITY = 0.2;
    private static final double MAX_DISTANCE = 4.0;
    private static final double MIN_CONFIDENCE = 0.7;
    private static final AlgaeTracker _algaeTracker = AlgaeTracker.getInstance();
    private static final CoralTracker _coralTracker = CoralTracker.getInstance();

    public VisionSubsystem(RobotContainer container, VisionIO io) {
        super(container, io, new VisionInputs(), new VisionOutputs());
        setPipelineIndex("limelight-center", 0);
        setPipelineIndex("limelight-left", 0);
        setPipelineIndex("South_Camera", 1);
    }

    public static boolean isValidTag(AprilTagObservation observation) {
        if (observation == null) return false;

        boolean isValidId = false;
        for (int id : FieldConstants.Reef.blueAllianceTagIds) {
            if (id == observation.getId()) {
                isValidId = true;
                break;
            }
        }
        if (!isValidId) return false;

        double latencyMs = observation.getLatency() * 1000.0;
        double ambiguity = observation.getAmbiguity();
        double distance = observation.get2dDistance();
        double confidence = observation.getConfidence();

        return ambiguity < MAX_AMBIGUITY
                && latencyMs < MAX_LATENCY_MS
                && distance < MAX_DISTANCE
                && confidence >= MIN_CONFIDENCE;
    }

    public List<AprilTagObservation> getValidTags() {
        List<AprilTagObservation> validTags = new ArrayList<>();
        for (List<AprilTagObservation> observations : _inputs.cameraAprilTagObservations.values()) {
            for (AprilTagObservation observation : observations) {
                if (isValidTag(observation)) {
                    validTags.add(observation);
                }
            }
        }
        return validTags;
    }

    @Override
    protected void periodic(VisionInputs inputs, VisionOutputs outputs) {
        List<Pose2d> robotPoses = new ArrayList<>();
        List<RobotPoseEstimate> estimates = new ArrayList<>();
        for (var poseEstimate : inputs.estimatedPoses.values()) {
            estimates.add(poseEstimate);
            Pose2d pose = poseEstimate.getPose3d().toPose2d();
            robotPoses.add(pose);
            RobotStateManager.getInstance().updateVision(poseEstimate);
        }

        log("Pose Estimates", estimates, RobotPoseEstimate.struct, Importance.CRITICAL);
        log("VisionPoses", robotPoses, Pose2d.struct, Importance.CRITICAL);

        List<AprilTagObservation> northTags = filterValidTags(getNorthCameraObservations());
        List<AprilTagObservation> northWestTags = filterValidTags(getNorthWestCameraObservations());

        for (var tag : northTags) {
            log("Distance", calculateDistanceWithCalibration(tag, "limelight-center"));
        }

        log("NorthTags", northTags, AprilTagObservation.struct);
        log("NorthWestTags", northWestTags, AprilTagObservation.struct);

        List<Pose2d> neuralDetections = new ArrayList<>();
        for (var observations : inputs.cameraNeuralPipelineObservations.values()) {
            for (var obs : observations) {
                neuralDetections.add(
                        new Pose2d(
                                obs.getX(),
                                obs.getY(),
                                obs.getClassId() == 0
                                        ? Rotation2d.kZero
                                        : Rotation2d.kCW_90deg)); // visually see the difference between coral and algae
            }
        }
        log("Raw Neural Detections", neuralDetections, Pose2d.struct, Importance.CRITICAL);
        _algaeTracker.update(inputs.cameraNeuralPipelineObservations.get("limelight-center"));
        _coralTracker.update(inputs.cameraNeuralPipelineObservations.get("South_Camera"));
    }

    private List<AprilTagObservation> filterValidTags(List<AprilTagObservation> observations) {
        return observations.stream().filter(VisionSubsystem::isValidTag).collect(Collectors.toList());
    }

    public boolean hasValidTargets() {
        return !getValidTags().isEmpty();
    }

    public List<AprilTagObservation> getNorthCameraObservations() {
        return _inputs.cameraAprilTagObservations.getOrDefault("limelight-center", new ArrayList<>());
    }

    public List<AprilTagObservation> getNorthWestCameraObservations() {
        return _inputs.cameraAprilTagObservations.getOrDefault("limelight-left", new ArrayList<>());
    }

    public Optional<AprilTagObservation> getTagFromObservations(
            List<AprilTagObservation> observations, int tagId) {
        return observations.stream().filter(tag -> (tag.getId() == tagId)).findFirst();
    }

    public AprilTagObservation getClosestTag() {
        List<AprilTagObservation> validTags = getValidTags();
        if (validTags.isEmpty()) return null;

        return validTags.stream()
                .max(Comparator.comparingDouble(AprilTagObservation::getArea))
                .orElse(null);
    }

    public AprilTagObservation getBestTag() {
        List<AprilTagObservation> validTags = getValidTags();
        if (validTags.isEmpty()) return null;

        return validTags.stream()
                .min(Comparator.comparingDouble(AprilTagObservation::getAmbiguity))
                .orElse(null);
    }

    public double calculateLateralOffset(AprilTagObservation observation, String cameraName) {
        TargetCorners corners = observation.getCorners();

        // Calculate center X coordinate from all four corners
        double centerX =
                (corners.topLeft.getX()
                                + corners.topRight.getX()
                                + corners.bottomLeft.getX()
                                + corners.bottomRight.getX())
                        / 4.0;

        Matrix<N3, N3> calibrationMatrix = getCalibrationMatrix(cameraName);
        double principalX = calibrationMatrix.get(0, 2);

        return (centerX - principalX) / principalX;
    }

    public double calculateDistanceWithCalibration(
            AprilTagObservation observation, String cameraName) {

        Matrix<N3, N3> calibrationMatrix = getCalibrationMatrix(cameraName);
        TargetCorners corners = observation.getCorners();

        // Focal length from calibration matrix
        double fx = calibrationMatrix.get(0, 0);
        double fy = calibrationMatrix.get(1, 1);
        double focalLength = (fx + fy) / 2.0;

        // Calculate average pixel size of the tag
        double pixelWidth1 = corners.topRight.getDistance(corners.topLeft); // Top
        double pixelWidth2 = corners.bottomRight.getDistance(corners.bottomLeft); // Bottom
        double pixelHeight1 = corners.topLeft.getDistance(corners.bottomLeft); // Left
        double pixelHeight2 = corners.topRight.getDistance(corners.bottomRight); // Right

        double avgPixelSize = (pixelWidth1 + pixelWidth2 + pixelHeight1 + pixelHeight2) / 4.0;

        // Similar triangles: distance = (actual_tag_size * focal_length) / pixel_size
        return Units.inchesToMeters(6.5) * focalLength / avgPixelSize;
    }

    public double calculateHorizontalFOV(Matrix<N3, N3> calibrationMatrix, double imageWidth) {
        return 2 * Math.atan(imageWidth / (2 * calibrationMatrix.get(0, 0)));
    }

    public double calculateVerticalFOV(Matrix<N3, N3> calibrationMatrix, double imageHeight) {
        return 2 * Math.atan(imageHeight / (2 * calibrationMatrix.get(1, 1)));
    }

    public double calculateDistanceFromAngles(AprilTagObservation observation, String cameraName) {
        final double TAG_SIZE_METERS = Units.inchesToMeters(6.5);

        Transform3d cameraToRobot;

        if (cameraName.equals("limelight-center")) {
            cameraToRobot = Constants.Vision.ROBOT_TO_NE_CAM;
        } else if (cameraName.equals("limelight-left")) {
            cameraToRobot = Constants.Vision.ROBOT_TO_NW_CAM;
        } else {
            System.err.println("Invalid camera " + cameraName);
            // Default values if camera not recognized
            cameraToRobot = new Transform3d();
        }

        return calculateDistanceFromAngles(observation, cameraToRobot, TAG_SIZE_METERS);
    }

    public double calculateDistanceFromAngles(
            AprilTagObservation observation, Transform3d cameraToRobot, double tagSizeMeters) {

        Optional<Pose3d> tagPose = FieldConstants.aprilTagLayout.getTagPose(observation.getId());
        if (tagPose.isEmpty()) {
            return -1;
        }

        double tyRadians = Math.toRadians(observation.getTy());
        Rotation3d cameraMountAngle = cameraToRobot.getRotation();
        double adjustedTy = tyRadians + cameraMountAngle.getY();

        return (tagPose.get().getZ() - cameraToRobot.getZ()) / (Math.tan(adjustedTy));
    }

    private Matrix<N3, N3> getCalibrationMatrix(String cameraName) {
        if (RobotBase.isSimulation()) {
            return Constants.Vision.simCalibrationMatrix;
        }

        if (cameraName.equals("limelight-center")) {
            return Constants.Vision.NORTH_CALIB_MATRIX;
        } else if (cameraName.equals("limelight-left")) {
            return Constants.Vision.NORTH_WEST_CALIB_MATRIX;
        } else {
            System.err.println("Invalid camera " + cameraName);
            return Matrix.eye(Nat.N3());
        }
    }

    @Override
    protected void processInputs() {}

    public void setPipelineIndex(String cameraName, int pipelineIndex) {
        _outputs.targetPipelines.put(cameraName, pipelineIndex);
    }

    public Optional<NeuralPipelineObservation> getClosestNeuralObservationOfType(
            String cameraName, int classId) {
        Optional<NeuralPipelineObservation> res = Optional.empty();
        var dist = Double.POSITIVE_INFINITY;
        for (var obs : _inputs.cameraNeuralPipelineObservations.get(cameraName)) {
            if (obs.getClassId() == classId && obs.getDistance() < dist) {
                res = Optional.of(obs);
                dist = obs.getDistance();
            }
        }
        return res;
    }
}
