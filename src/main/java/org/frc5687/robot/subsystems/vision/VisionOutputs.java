package org.frc5687.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import java.util.HashMap;
import java.util.Map;
import org.frc5687.robot.util.BaseOutputs;

public class VisionOutputs extends BaseOutputs {
    @Logged
    public boolean hack = true; // has to have something logged if there are fields it looks like ..?

    @NotLogged public Map<String, Integer> targetPipelines = new HashMap<>();
}
