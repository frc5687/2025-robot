package org.frc5687.robot.subsystems.lights;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.RobotStateManager;
import org.frc5687.robot.subsystems.OutliersSubsystem;
import org.frc5687.robot.subsystems.SubsystemIO;
import org.frc5687.robot.subsystems.intake.IntakeInputs;
import org.frc5687.robot.subsystems.intake.IntakeOutputs;
import org.frc5687.robot.subsystems.intake.IntakeState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class LightSubsystem extends OutliersSubsystem<LightInputs, LightOutputs> {

    public LightSubsystem(RobotContainer container, SubsystemIO<LightInputs, LightOutputs> io) {
        super(container, io, new LightInputs(), new LightOutputs());
        setDesiredState(LightState.PLACEHOLDER);
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
