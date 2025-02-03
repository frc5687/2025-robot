package org.frc5687.robot.subsystems.vision;

import org.photonvision.PhotonCamera;

public class PhotonVisionIO implements VisionIO {

    private final String _name;
    private final PhotonCamera _camera;

    public PhotonVisionIO(String name) {
        _name = name;
        _camera = new PhotonCamera(_name);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {}

    @Override
    public void writeOutputs(VisionOutputs Outputs) {}
}
