package org.frc5687.robot.subsystems.coralarm;

import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class CoralArmSubsystem extends OutliersSubsystem<CoralInputs, CoralOutputs> {

    private final RobotStateManager _stateManger = RobotStateManager.getInstance();

    public CoralArmSubsystem(CoralArmIO io) {
        super(io, new CoralInputs(), new CoralOutputs());
    }

    @Override
    protected void processInputs() {}

    @Override
    protected void periodic(CoralInputs inputs, CoralOutputs outputs) {
        _stateManger.updateCoralArm(_inputs.angleRads);
        _inputs.pose = _stateManger.getPose(RobotCoordinate.CORAL_ARM_BASE);
    }

    public void setDesiredState(CoralState state) {
        _outputs.desiredState = state;
        setDesiredAngleRadians(state.getValue());
    }

    public CoralState getDesiredState() {
        return _outputs.desiredState;
    }

    public void setDesiredAngleRadians(double angleRadians) {
        _outputs.desiredAngleRad = angleRadians;
    }

    public double getArmAngleRads() {
        return _inputs.angleRads;
    }

    public void setArmAngle(double angle) {
        _outputs.desiredAngleRad = angle;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.001;
    }

    public void setCoralMotorVoltage(double voltage) {
        _outputs.voltageCommand = voltage;
    }

    public void setCurrentState(CoralState state) {
        _inputs.currentState = state;
    }

    public CoralState getCurrentState() {
        return _inputs.currentState;
    }

    public void mapToClosestState() {
        CoralState closestState = CoralState.STOWED;
        double min_dist = Double.MAX_VALUE;
        for (CoralState state : CoralState.values()) {
            double dist = Math.abs(getArmAngleRads() - state.getValue());
            if (dist < min_dist) {
                closestState = state;
                min_dist = dist;
            }
        }
        _inputs.currentState = closestState;
    }
}
