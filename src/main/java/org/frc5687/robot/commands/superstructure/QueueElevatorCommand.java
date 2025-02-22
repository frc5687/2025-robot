package org.frc5687.robot.commands.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.frc5687.robot.commands.OutliersCommand;

public class QueueElevatorCommand extends OutliersCommand {

    Supplier<Double> _poseTMinus;
    Supplier<Double> _elevatorSignedTMinus;
    Command _elevatorCommand;
    boolean _isFinished;

    public QueueElevatorCommand(
            Supplier<Double> _poseTMinus,
            Supplier<Double> _elevatorSignedTMinus,
            Command _elevatorCommand) {
        super();
        this._poseTMinus = _poseTMinus;
        this._elevatorSignedTMinus = _elevatorSignedTMinus;
        this._elevatorCommand = _elevatorCommand;
    }

    @Override
    public void initialize() {
        _isFinished = false;
    }

    @Override
    protected void execute(double timestamp) {
         
        // very scuffed sorry - xavier
        double elevatorSignedTMinus = _elevatorSignedTMinus.get();
        double poseTMinus = _poseTMinus.get();
        log("elevatorTMinus", elevatorSignedTMinus);
        log("poseTMinus", poseTMinus);
        if (elevatorSignedTMinus < 0.0 && Math.abs(elevatorSignedTMinus) >= poseTMinus) {
            // it's time!!!
            _elevatorCommand.schedule();
            _isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return _isFinished;
    }
}
