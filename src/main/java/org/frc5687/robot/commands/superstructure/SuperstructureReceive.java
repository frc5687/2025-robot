package org.frc5687.robot.commands.superstructure;

import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;

public class SuperstructureReceive extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final CoralArmSubsystem _coral;
    private final AlgaeArmSubsystem _algae;
    private final IntakeSubsystem _intake;
    private final SuperstructureState _goal;

    public SuperstructureReceive(RobotContainer container, SuperstructureState goal) {
        _elevator = container.getElevator();
        _coral = container.getCoral();
        _algae = container.getAlgae();
        _intake = container.getIntake();
        _goal = goal;

        addRequirements(_elevator, _coral, _algae, _intake);
    }

    @Override
    public void initialize() {
        _elevator.setDesiredState(_goal.getElevator());
        _coral.setDesiredState(_goal.getCoral());
        _algae.setDesiredState(_goal.getAlgae());
        _intake.setDesiredState(_goal.getIntake());
    }

    @Override
    public void execute(double timestamp) {
        _coral.setCoralMotorVoltage(6.0);
    }

    @Override
    public boolean isFinished() {
        return _coral.isCoralDetected();
    }

    @Override
    public void end(boolean interrupted) {
        _coral.setCoralMotorVoltage(0.0);
    }
}
