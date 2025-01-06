package org.frc5687.robot.util;

import edu.wpi.first.epilogue.Logged;

/*
 * Base oututs or the DESIRED state from a system:
 * Any CONTROLLABLE quantity we send to hardware
 * The COMMANDS we send to actuators
 * What we WANT the hardware to do
 * Raw commands to devices
 * 
 * Outputs are ONLY things we can control:
 * Motor voltages
 * Target positions
 * Solenoid states
 * Current limits
 * Control modes
 * etc.
 */

public abstract class BaseOutputs {
    public String logRootName = "";

    public void setLogRootName(String name) {
        logRootName = name;
    }

    public String getLogPath() {
        return logRootName.isEmpty() ? getClass().getSimpleName() : logRootName;
    }
}