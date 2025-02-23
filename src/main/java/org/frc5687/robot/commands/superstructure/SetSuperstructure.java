package org.frc5687.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Optional;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.algaearm.AlgaeState;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;
import org.frc5687.robot.subsystems.coralarm.CoralState;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.intake.IntakeState;
import org.frc5687.robot.subsystems.intake.IntakeSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;

public class SetSuperstructure extends OutliersCommand {

    private final ElevatorSubsystem _elevator;
    private final CoralArmSubsystem _coral;
    private final AlgaeArmSubsystem _algae;
    private final IntakeSubsystem _intake;
    private final RobotContainer _container;
    private Optional<ElevatorState> _elevatorGoal;
    private Optional<AlgaeState> _algaeGoal;
    private Optional<CoralState> _coralGoal;
    private Optional<IntakeState> _intakeGoal;

    private Command _command;

    public SetSuperstructure(RobotContainer container, SuperstructureState goalState) {
        this(
                container,
                Optional.of(goalState.getAlgae()),
                Optional.of(goalState.getElevator()),
                Optional.of(goalState.getIntake()),
                Optional.of(goalState.getCoral()));
    }

    public SetSuperstructure(
            RobotContainer container,
            Optional<AlgaeState> algaeGoal,
            Optional<ElevatorState> elevatorGoal,
            Optional<IntakeState> intakeGoal,
            Optional<CoralState> coralGoal) {
        _elevator = container.getElevator();
        _coral = container.getCoral();
        _algae = container.getAlgae();
        _intake = container.getIntake();
        _container = container;
        _elevatorGoal = elevatorGoal;
        _algaeGoal = algaeGoal;
        _coralGoal = coralGoal;
        _intakeGoal = intakeGoal;
    }

    private boolean willCollide() {
        if (_elevatorGoal.isEmpty()) {
            return false; // no elevator movement,, will not collide :)  (huge win)
        }
        double prevHeight = _elevator.getInputs().motorHeightMeters;
        double newHeight = _elevatorGoal.get().getHeight();
        return Math.abs(newHeight - prevHeight) > 0.02;
    }

    @Override
    public void initialize() {
        if (willCollide()) {
            var step1 =
                    new SetSuperstructureRaw(
                            _container,
                            Optional.empty(),
                            Optional.empty(),
                            Optional.empty(),
                            Optional.of(CoralState.IDLE));
            var step2 =
                    new SetSuperstructureRaw(
                            _container, _algaeGoal, _elevatorGoal, _intakeGoal, Optional.of(CoralState.IDLE));
            var step3 =
                    new SetSuperstructureRaw(_container, _algaeGoal, _elevatorGoal, _intakeGoal, _coralGoal);
            System.out.println("will  collide");
            _command = new SequentialCommandGroup(step1, step2, step3);
        } else {
            System.out.println("will not collide");
            _command =
                    new SetSuperstructureRaw(_container, _algaeGoal, _elevatorGoal, _intakeGoal, _coralGoal);
        }
        _command.initialize();
    }

    @Override
    protected void execute(double timestamp) {
        _command.execute();
    }

    @Override
    public boolean isFinished() {
        return _command.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        _command.end(interrupted);
    }
}
