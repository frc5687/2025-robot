package org.frc5687.robot.commands.superstructure;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;

public class WaitUntilAlgaeDetected extends OutliersCommand {
    private final AlgaeArmSubsystem _algae;

    public WaitUntilAlgaeDetected(RobotContainer container) {
        _algae = container.getAlgae();
        // addRequirements(_algae); i think this is not needed - xavier
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _algae.isAlgaeDetected();
    }

    @Override
    public void end(boolean interrupted) {}
}
