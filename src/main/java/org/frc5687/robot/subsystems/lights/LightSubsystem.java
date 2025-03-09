package org.frc5687.robot.subsystems.lights;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;

public class LightSubsystem extends OutliersSubsystem<LightInputs, LightOutputs> {
    private final RobotContainer _container;

    public LightSubsystem(RobotContainer container, SubsystemIO<LightInputs, LightOutputs> io) {
        super(container, io, new LightInputs(), new LightOutputs());
        _container = container;
    }

    @Override
    protected void processInputs() {
        RobotStateManager.getInstance();
    }

    @Override
    protected void periodic(LightInputs inputs, LightOutputs outputs) {
        if (_container.getClimber().isSensorTriggered()) {
            outputs.desiredState = LightState.BLUE;
        } else if (_container.getSuperstructureManager().isAlgaeMode()) {
            if (_container.getAlgae().isAlgaeDetected()) {
                outputs.desiredState = LightState.FLASHING_GREEN;
            } else {
                outputs.desiredState = LightState.SOLID_GREEN;
            }
        } else { // coral mode
            if (_container.getCoral().isCoralDetected()) {
                outputs.desiredState = LightState.FLASHING_WHITE;
            } else {
                outputs.desiredState = LightState.SOLID_WHITE;
            }
        }
    }
}
