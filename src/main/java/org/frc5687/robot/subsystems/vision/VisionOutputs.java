package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import org.frc5687.robot.util.BaseOutputs;

public class VisionOutputs extends BaseOutputs {
    @Logged(name = "Target Pipeline Index", importance = Logged.Importance.DEBUG)
    public int pipelineIndex = 0;
}
