package org.frc5687.robot.subsystems.coralarm;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class CoralArmSubsystem extends OutliersSubsystem<CoralInputs, CoralOutputs> {

    private final RobotStateManager _stateManger = RobotStateManager.getInstance();

    public CoralArmSubsystem(RobotContainer container, CoralArmIO io) {
        super(container, io, new CoralInputs(), new CoralOutputs());
        setDesiredState(_inputs.currentState);
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
        setDesiredAngleRadians(state.getArmAngle());
        setWheelVoltageCommand(state.getRollerVoltage());
    }

    public void setWheelVoltageCommand(double voltage) {
        _outputs.wheelVoltageCommand = voltage;
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
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.05;
    }

    public boolean isCoralDetected() {
        return _inputs.isCoralDetected;
    }

    public void setCurrentState(CoralState state) {
        _inputs.currentState = state;
    }

    public CoralState getCurrentState() {
        return _inputs.currentState;
    }

    public void mapToClosestState() {
        CoralState closestState = CoralState.STOWED;
        double minDist = Double.MAX_VALUE;
        for (CoralState state : CoralState.values()) {
            double dist = Math.abs(getArmAngleRads() - state.getArmAngle());
            if (dist < minDist) {
                closestState = state;
                minDist = dist;
            }
        }
        _inputs.currentState = closestState;
    }
}
