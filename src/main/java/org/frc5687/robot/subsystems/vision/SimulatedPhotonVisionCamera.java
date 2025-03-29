package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class SimulatedPhotonVisionCamera extends PhotonVisionCamera {

    private final VisionSystemSim _visionSim;
    private final PhotonCameraSim _cameraSim;

    public SimulatedPhotonVisionCamera(String name, Transform3d robotToCamera, VisionSystemSim sim) {
        super(name, robotToCamera);
        _visionSim = sim;

        SimCameraProperties cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(1280, 720, Rotation2d.fromDegrees(88));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(60);
        cameraProp.setAvgLatencyMs(20);
        cameraProp.setLatencyStdDevMs(5);

        _cameraSim = new PhotonCameraSim(_cam, cameraProp);
        _cameraSim.enableDrawWireframe(true);
        _cameraSim.enableProcessedStream(true);
        _cameraSim.enableRawStream(true);
        _cameraSim.setMaxSightRange(8.0);

        _visionSim.addCamera(_cameraSim, robotToCamera);
    }
}
