package org.frc5687.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Map;
import org.frc5687.robot.Constants;
import org.frc5687.robot.subsystems.vision.Camera.VisionResult;

public class HardwareVisionIO implements VisionIO {

    private final Map<String, Camera> _cams;

    public HardwareVisionIO() {
        _cams = new HashMap<>();
        _cams.put("limelight-left", new LimelightCamera("limelight-left"));
        _cams.put("limelight-center", new LimelightCamera("limelight-center"));
        _cams.put(
                "South_Camera",
                new PhotonVisionCamera("South_Camera", Constants.Vision.ROBOT_TO_SOUTH_CAM));
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
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
