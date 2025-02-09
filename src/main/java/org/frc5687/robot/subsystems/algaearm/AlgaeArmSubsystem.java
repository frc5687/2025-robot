package org.frc5687.robot.subsystems.algaearm;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class AlgaeArmSubsystem extends OutliersSubsystem<AlgaeInputs, AlgaeOutputs> {

    private final RobotStateManager _stateManger = RobotStateManager.getInstance();

    public AlgaeArmSubsystem(RobotContainer container, AlgaeArmIO io) {
        super(container, io, new AlgaeInputs(), new AlgaeOutputs());
        setDesiredState(_inputs.algaeState);
    }

    @Override
    protected void processInputs() {}

    @Override
    protected void periodic(AlgaeInputs inputs, AlgaeOutputs outputs) {
        _stateManger.updateAlgaeArm(_inputs.angleRads);
        _inputs.pose = _stateManger.getPose(RobotCoordinate.ALGAE_ARM_BASE);
    }

    public AlgaeState getDesiredState() {
        return _outputs.desiredState;
    }

    public void setDesiredState(AlgaeState state) {
        _outputs.desiredState = state;
        setDesiredAngleRadians(state.getValue());
    }

    public void setDesiredAngleRadians(double angleRadians) {
        _outputs.desiredAngleRad = angleRadians;
    }

    public void setArmAngle(double angle) {
        _outputs.desiredAngleRad = angle;
    }

    public double getArmAngleRads() {
        return _inputs.angleRads;
    }

    public void setAlgaeMotorVoltage(double voltage) {
        _outputs.voltageCommand = voltage;
    }

    // TODO: Make tolerance for all subsystems
    public boolean isAtDesiredAngle() {
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.01;
    }

    public void setCurrentState(AlgaeState state) {
        _inputs.algaeState = state;
    }

    public AlgaeState getCurrentState() {
        return _inputs.algaeState;
    }

    public void mapToClosestState() {
        AlgaeState closestState = AlgaeState.IDLE;
        double minDist = Double.MAX_VALUE;
        for (AlgaeState state : AlgaeState.values()) {
            double dist = Math.abs(getArmAngleRads() - state.getValue());
            if (dist < minDist) {
                closestState = state;
                minDist = dist;
            }
        }
        _inputs.algaeState = closestState;
    }
}
