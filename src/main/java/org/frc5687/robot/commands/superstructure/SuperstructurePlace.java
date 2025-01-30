package org.frc5687.robot.commands.superstructure;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class SuperstructurePlace extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final CoralArmSubsystem _coral;
    private final AlgaeArmSubsystem _algae;

    public SuperstructurePlace(
            ElevatorSubsystem elevator, CoralArmSubsystem coral, AlgaeArmSubsystem algae) {
        _elevator = elevator;
        _coral = coral;
        _algae = algae;
        addRequirements(_elevator, _coral, _algae);
    }

    @Override
    public void initialize() {
        super.initialize();
        _elevator.setDesiredPlatformHeightWorld(_elevator.getCurrentState().getValue() - 0.2);
        _algae.setDesiredState(AlgaeState.IDLE);
    }

    @Override
    protected void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _elevator.isAtDesiredPosition() && _algae.isAtDesiredAngle();
    }

    @Override
    public void end(boolean interrupted) {
        _elevator.mapToClosestState();
        super.end(interrupted);
    }
}
