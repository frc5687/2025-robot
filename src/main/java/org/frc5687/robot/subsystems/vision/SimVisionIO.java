package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.util.HashMap;
import java.util.Map;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.vision.Camera.VisionResult;
import org.frc5687.robot.util.FieldConstants;
import org.photonvision.simulation.VisionSystemSim;

public class SimVisionIO implements VisionIO {

    private final Map<String, Camera> _cams;
    private final VisionSystemSim _visionSim;

    public SimVisionIO() {
        _visionSim = new VisionSystemSim("MainVision");
        AprilTagFieldLayout layout = null;
        layout = FieldConstants.aprilTagLayout;
        _visionSim.addAprilTags(layout);

        _cams = new HashMap<>();
        _cams.put(
                "limelight-left",
                new SimulatedPhotonVisionCamera(
                        "limelight-left", Constants.Vision.ROBOT_TO_NW_CAM, _visionSim));
        _cams.put(
                "limelight-center",
                new SimulatedPhotonVisionCamera(
                        "limelight-center", Constants.Vision.ROBOT_TO_NE_CAM, _visionSim));
        _cams.put(
                "South_Camera",
                new SimulatedPhotonVisionCamera(
                        "South_Camera", Constants.Vision.ROBOT_TO_SOUTH_CAM, _visionSim));
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        _visionSim.update(
                RobotStateManager.getInstance().getPose(RobotCoordinate.ROBOT_BASE_ODOM).toPose2d());

        inputs.cameraAprilTagObservations.clear();
        inputs.cameraNeuralPipelineObservations.clear();
        inputs.estimatedPoses.clear();

        for (var entry : _cams.entrySet()) {
            String name = entry.getKey();
            Camera cam = entry.getValue();
            VisionResult res = cam.getLatestResult();
            inputs.cameraAprilTagObservations.put(name, res.tags());
            inputs.cameraNeuralPipelineObservations.put(name, res.neuralObservations());
            if (res.poseEstimate().isPresent()) {
                inputs.estimatedPoses.put(name, res.poseEstimate().get());
            }
        }
    }

    @Override
    public void writeOutputs(VisionOutputs outputs) {
        for (var entry : outputs.targetPipelines.entrySet()) {
            _cams.get(entry.getKey()).setPipelineIndex(entry.getValue());
        }
    }
}
