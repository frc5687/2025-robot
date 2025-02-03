package org.frc5687.robot.util;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.util.struct.Struct;
import java.util.Collection;

public interface EpilogueLog {
    String getLogBase();

    default String getLogPath(String identifier) {
        return getLogBase() + "/" + identifier;
    }

    default void log(String identifier, int value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, long value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, float value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, double value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, boolean value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, String value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, byte[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, int[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, long[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, float[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, double[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, boolean[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default void log(String identifier, String[] value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default <E extends Enum<E>> void log(String identifier, E value) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value);
    }

    default <S> void log(String identifier, S value, Struct<S> struct) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
    }

    default <S> void log(String identifier, S[] value, Struct<S> struct) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
    }

    default <S> void log(String identifier, Collection<S> value, Struct<S> struct) {
        Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
    }
}
