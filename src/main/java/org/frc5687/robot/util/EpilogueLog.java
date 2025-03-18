package org.frc5687.robot.util;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Collection;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.LongSupplier;

public interface EpilogueLog {
    String getLogBase();

    default String getLogPath(String identifier) {
        return getLogBase() + "/" + identifier;
    }

    default boolean shouldLog(Logged.Importance importance) {
        return Epilogue.shouldLog(importance);
    }

    default void log(String identifier, int value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, long value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, float value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, double value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, boolean value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, String value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, int value, Logged.Importance importance) {
        if (shouldLog(importance)) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, long value, Logged.Importance importance) {
        if (shouldLog(importance)) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, float value, Logged.Importance importance) {
        if (shouldLog(importance)) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, double value, Logged.Importance importance) {
        if (shouldLog(importance)) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, boolean value, Logged.Importance importance) {
        if (shouldLog(importance)) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, String value, Logged.Importance importance) {
        if (shouldLog(importance)) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, byte[] value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, int[] value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, long[] value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, float[] value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, double[] value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, boolean[] value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, String[] value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, byte[] value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, int[] value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, long[] value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, float[] value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, double[] value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, boolean[] value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default void log(String identifier, String[] value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default <E extends Enum<E>> void log(String identifier, E value) {
        log(identifier, value, Logged.Importance.DEBUG);
    }

    default <E extends Enum<E>> void log(String identifier, E value, Logged.Importance importance) {
        if (shouldLog(importance) && value != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default <S> void log(String identifier, S value, Struct<S> struct) {
        log(identifier, value, struct, Logged.Importance.DEBUG);
    }

    default <S> void log(String identifier, S value, Struct<S> struct, Logged.Importance importance) {
        if (shouldLog(importance) && value != null && struct != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default <S> void log(String identifier, S[] value, Struct<S> struct) {
        log(identifier, value, struct, Logged.Importance.DEBUG);
    }

    default <S> void log(
            String identifier, S[] value, Struct<S> struct, Logged.Importance importance) {
        if (shouldLog(importance) && value != null && struct != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default <S> void log(String identifier, Collection<S> value, Struct<S> struct) {
        log(identifier, value, struct, Logged.Importance.DEBUG);
    }

    default <S> void log(
            String identifier, Collection<S> value, Struct<S> struct, Logged.Importance importance) {
        if (shouldLog(importance) && value != null && struct != null) {
            try {
                Epilogue.getConfig().backend.log(getLogPath(identifier), value, struct);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, BooleanSupplier supplier) {
        log(identifier, supplier, Logged.Importance.DEBUG);
    }

    default void log(String identifier, BooleanSupplier supplier, Logged.Importance importance) {
        if (shouldLog(importance) && supplier != null) {
            try {
                log(identifier, supplier.getAsBoolean(), importance);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, IntSupplier supplier) {
        log(identifier, supplier, Logged.Importance.DEBUG);
    }

    default void log(String identifier, IntSupplier supplier, Logged.Importance importance) {
        if (shouldLog(importance) && supplier != null) {
            try {
                log(identifier, supplier.getAsInt(), importance);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, LongSupplier supplier) {
        log(identifier, supplier, Logged.Importance.DEBUG);
    }

    default void log(String identifier, LongSupplier supplier, Logged.Importance importance) {
        if (shouldLog(importance) && supplier != null) {
            try {
                log(identifier, supplier.getAsLong(), importance);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void log(String identifier, DoubleSupplier supplier) {
        log(identifier, supplier, Logged.Importance.DEBUG);
    }

    default void log(String identifier, DoubleSupplier supplier, Logged.Importance importance) {
        if (shouldLog(importance) && supplier != null) {
            try {
                log(identifier, supplier.getAsDouble(), importance);
            } catch (Exception e) {
                handleLoggingError(identifier, e);
            }
        }
    }

    default void handleLoggingError(String identifier, Exception e) {
        DriverStation.reportError(
                String.format("%s failed to log with exception: %s", identifier, e.getMessage()), false);
    }
}
