package org.frc5687.robot.subsystems.lights;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class LightSubsystem extends OutliersSubsystem<LightInputs, LightOutputs> {

    public LightSubsystem(RobotContainer container, SubsystemIO<LightInputs, LightOutputs> io) {
        super(container, io, new LightInputs(), new LightOutputs());
        setDesiredState(LightState.TWINKLE_DEFAULT);
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance();
    }

    @Override
    protected void periodic(LightInputs inputs, LightOutputs outputs) {}

    public void setDesiredState(LightState state) {
        _outputs.desiredState = state;
    }
}
