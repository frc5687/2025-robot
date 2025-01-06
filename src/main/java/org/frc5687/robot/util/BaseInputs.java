package org.frc5687.robot.util;

import edu.wpi.first.epilogue.Logged;

/*
 * Inputs for our subsystems are the ACTUAL measurement from a system
 * 
 * Any MEASURABLE quantity from hardware (sensors, encoders, motor controllers)
 * The CURRENT STATE of the hardware
 * What the hardware is ACTUALLY doing
 * Raw data that needs to be processed
 * 
 * Inputs are ONLY things we can measure:
 * Sensor readings
 * Encoder positions
 * Current measurements
 * Temperature readings
 * Limit switch states
 * Fault codes
 * etc.
 */

public abstract class BaseInputs {
    public int faults = 0;
    public String logRootName = "";

    public void setLogRootName(String name) {
        logRootName = name;
    }

    public String getLogPath() {
        return logRootName.isEmpty() ? getClass().getSimpleName() : logRootName;
    }
}
