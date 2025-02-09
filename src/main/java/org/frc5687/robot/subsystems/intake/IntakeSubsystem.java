package org.frc5687.robot.subsystems.intake;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class IntakeSubsystem extends OutliersSubsystem<IntakeInputs, IntakeOutputs> {

    public IntakeSubsystem(RobotContainer container, SubsystemIO<IntakeInputs, IntakeOutputs> io) {
        super(container, io, new IntakeInputs(), new IntakeOutputs());
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance().updateIntakeArm(_inputs.angleRads);
    }

    @Override
    protected void periodic(IntakeInputs inputs, IntakeOutputs outputs) {}

    public void setDesiredState(IntakeState state) {
        _outputs.desiredAngleRad = state.getValue();
    }

    public void setCurrentState(IntakeState state) {
        _inputs.currentState = state;
    }

    public void setRollerVoltage(double voltage) {
        _outputs.rollerVoltage = voltage;
    }

    public void setIntakeVoltage(double voltage) {
        _outputs.intakeVoltage = voltage;
    }

    public void setPivotAngle(double angle) {
        _outputs.pivotTargetAngle = angle;
    }

    public double getPivotArmAngleRads() {
        return _inputs.angleRads;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(_inputs.angleRads - _outputs.desiredAngleRad) < 0.01;
    }

    // This is to map to closest state incase of interrupt.
    public void mapToClosestState() {
        IntakeState closestState = IntakeState.STOWED;
        double minDist = Double.MAX_VALUE;

        for (IntakeState state : IntakeState.values()) {
            double angleDiff = Math.abs(getPivotArmAngleRads() - state.getValue());
            if (angleDiff < minDist) {
                closestState = state;
                minDist = angleDiff;
            }
        }
        _inputs.currentState = closestState;
    }
}
