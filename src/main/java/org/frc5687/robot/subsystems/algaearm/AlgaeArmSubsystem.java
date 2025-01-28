package org.frc5687.robot.subsystems.algaearm;

import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.RobotStateManager.RobotCoordinate;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class AlgaeArmSubsystem extends OutliersSubsystem<AlgaeInputs, AlgaeOutputs> {

    private final RobotStateManager _stateManger = RobotStateManager.getInstance();

    public AlgaeArmSubsystem(AlgaeArmIO io) {
        super(io, new AlgaeInputs(), new AlgaeOutputs());
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

    public void setDesiredAngleRadians(double angleRadians) {
        _outputs.desiredAngleRad = angleRadians;
    }

    public void setArmAngle(double angle) {
        _outputs.desiredAngleRad = angle;
    }

    public void setAlgaeMotorVoltage(double voltage) {
        _outputs.voltageCommand = voltage;
    }
}
