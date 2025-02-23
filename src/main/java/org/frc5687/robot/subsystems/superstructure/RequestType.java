package org.frc5687.robot.subsystems.superstructure;

public enum RequestType {
    IMMEDIATE, // Execute immediately, interrupt current action
    QUEUED, // Wait for drive condition
    // AUTO_SEQUENCE,
}
