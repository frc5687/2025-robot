package org.frc5687.robot.commands.algae;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;

public class IntakeAlgaeWheels extends OutliersCommand {
    private final AlgaeArmSubsystem _algae;

    public IntakeAlgaeWheels(RobotContainer container) {
        _algae = container.getAlgae();
    }

    @Override
    public void initialize() {
        _algae.setWheelMotorVoltage(12);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _algae.isAlgaeDetected();
    }

    @Override
    public void end(boolean interrupted) {
        _algae.setWheelMotorVoltage(0);
    }
}
