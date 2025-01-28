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

    public CoralState getDesiredState() {
        return _outputs.desiredState;
    }

    public void setDesiredAngleRadians(double angleRadians) {
        _outputs.desiredAngleRad = angleRadians;
    }

    public void setArmAngle(double angle) {
        _outputs.desiredAngleRad = angle;
    }

    public boolean isAtDesiredAngle() {
        return Math.abs(_outputs.desiredAngleRad - _inputs.angleRads) < 0.001;
    }

    public void setAlgaeMotorVoltage(double voltage) {
        _outputs.voltageCommand = voltage;
    }
}
