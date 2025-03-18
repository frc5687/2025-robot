package org.frc5687.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.robot.util.EpilogueLog;

public abstract class OutliersCommand extends Command implements EpilogueLog {
    private final double _startTime;

    public OutliersCommand() {
        _startTime = Timer.getTimestamp();
    }

    @Override
    public String getLogBase() {
        return this.getName();
    }

    protected abstract void execute(double timestamp);

    @Override
    public final void execute() {
        double timestamp = Timer.getTimestamp() - _startTime;
        execute(timestamp);
    }
}
