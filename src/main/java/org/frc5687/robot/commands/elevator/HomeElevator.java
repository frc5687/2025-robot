package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class HomeElevator extends OutliersCommand {
    private final ElevatorSubsystem _elevator;

    private static final TunableDouble HOMING_VOLTAGE =
            new TunableDouble("HomeElevator", "homing voltage", -0.5);
    private static final TunableDouble CURRENT_THRESHOLD =
            new TunableDouble("HomeElevator", "current threshold", 1.0);

    private boolean _isFinished = false;

    public HomeElevator(ElevatorSubsystem elevator) {
        _elevator = elevator;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        _isFinished = false;
    }

    @Override
    public void execute(double timestamp) {
        _elevator.setVoltage(HOMING_VOLTAGE.get());
        boolean currentSpiked = _elevator.getAverageCurrent() > CURRENT_THRESHOLD.get();

        if (currentSpiked) {
            System.out.println("zeroed elevator");
            _elevator.setVoltage(0.0);
            _elevator.zeroElevator();
            _elevator.setDesiredHeight(ElevatorState.STOWED);
            _isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return _isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        _elevator.setVoltage(0.0);
    }
}
