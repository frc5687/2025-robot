package org.frc5687.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

// Yoink 254 retry confuration logic
public class CTREUtil {
    private static final int MAX_RETRIES = 10;

    private static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
        StatusCode statusCode = StatusCode.OK;

        for (int i = 0; i < MAX_RETRIES && statusCode != StatusCode.OK; i++) {
            statusCode = function.get();
        }

        if (statusCode != StatusCode.OK) {
            String errorMsg =
                    String.format(
                            "Error calling %s on CTRE device ID %d: %s", function, deviceId, statusCode);
            DriverStation.reportError(errorMsg, true);
        }

        return statusCode;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
    }

    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }
}
