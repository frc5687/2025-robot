package org.frc5687.robot.subsystems.superstructure;

import org.frc5687.robot.Constants;

// TODO: ADD INDENDENT EJECT CORAL AND ALAGE
public record MotorState(
        double coralSpeed, double algaeSpeed, double intakeSpeed, String description) {
    public static final MotorState STOPPED = new MotorState(0, 0, 0, "STOPPED");
    public static final MotorState INTAKE = new MotorState(0.5, 0, 0.7, "INTAKE");
    public static final MotorState EJECT = new MotorState(-0.5, 0, -0.7, "EJECT");
    public static final MotorState HOLD = new MotorState(0.0, 0, 0, "HOLD");
    public static final MotorState RECEIVE_FUNNEL =
            new MotorState(Constants.CoralArm.WHEEL_RECEIVE_CORAL_DUTY_CYCLE, 0, 0, "RECEIVE_FUNNEL");
    public static final MotorState ALGAE_INTAKE = new MotorState(0, 0.5, 0, "ALGAE_INTAKE");
}
