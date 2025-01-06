package org.frc5687.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public abstract class OutliersCommand extends Command {
    private final double _startTime;

    public OutliersCommand() {
        _startTime = Timer.getTimestamp();
    }
 
    protected abstract void execute(double timestamp);

    @Override
    public final void execute() {
        double timestamp = Timer.getTimestamp() - _startTime;
        execute(timestamp);
    }
}
