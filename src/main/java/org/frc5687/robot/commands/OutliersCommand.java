package org.frc5687.robot.commands;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

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

    private String getLogPath(String identifier) {
        return this.getName() + "/" + identifier;
    }

    public void log(String identifier, int value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, long value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, float value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, double value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, boolean value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, String value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, byte[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, int[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, long[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, float[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, double[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, boolean[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public void log(String identifier, String[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public <E extends Enum<E>> void log(String identifier, E value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    public <S> void log(String identifier, S value, Struct<S> struct) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
    }

    public <S> void log(String identifier, S[] value, Struct<S> struct) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
    }
}
