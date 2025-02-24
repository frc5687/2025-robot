package org.frc5687.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class IntakeSubsystem extends OutliersSubsystem<IntakeInputs, IntakeOutputs> {

    public IntakeSubsystem(RobotContainer container, SubsystemIO<IntakeInputs, IntakeOutputs> io) {
        super(container, io, new IntakeInputs(), new IntakeOutputs());
        setDesiredState(IntakeState.IDLE);
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance().updateIntakeArm(_inputs.armAngleRads);
    }

    @Override
    protected void periodic(IntakeInputs inputs, IntakeOutputs outputs) {}

    public void setDesiredState(IntakeState state) {
        _outputs.desiredState = state;
        setDesiredPivotAngle(state.getValue());
    }

    public void setDesiredPivotAngle(double angle) {
        double desiredAngleClamped =
                MathUtil.clamp(angle, Constants.Intake.MIN_ANGLE, Constants.Intake.MAX_ANGLE);
        _outputs.desiredAngleRad = desiredAngleClamped;
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

    public double getPivotArmAngleRads() {
        return _inputs.armAngleRads;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(
                        new Rotation2d(_inputs.armAngleRads)
                                .minus(new Rotation2d(_outputs.desiredAngleRad))
                                .getDegrees())
                < 5;
    }

    public boolean isIntakeCoralDetected() {
        return _inputs.isCoralDetected;
    }

    // This is to map to closest state incase of interrupt.
    public void mapToClosestState() {
        IntakeState closestState = IntakeState.IDLE;
        double minDist = Double.MAX_VALUE;

        for (IntakeState state : IntakeState.values()) {
            double angleDiff = Math.abs(getPivotArmAngleRads() - state.getValue());
            if (angleDiff < minDist) {
                closestState = state;
                minDist = angleDiff;
            }
        }
        setCurrentState(closestState);
    }
}
