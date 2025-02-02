package org.frc5687.robot.subsystems.intake;

import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class IntakeSubsystem extends OutliersSubsystem<IntakeInputs, IntakeOutputs> {

    public IntakeSubsystem(SubsystemIO<IntakeInputs, IntakeOutputs> io) {
        super(io, new IntakeInputs(), new IntakeOutputs());
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance().updateIntakeArm(_inputs.angleRads);
    }

    @Override
    protected void periodic(IntakeInputs inputs, IntakeOutputs outputs) {}

    public void setState(IntakeState state) {
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
}
