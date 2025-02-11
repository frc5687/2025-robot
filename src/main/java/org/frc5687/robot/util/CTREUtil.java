package org.frc5687.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class CTREUtil {
    private static final int MAX_RETRIES = 5;

    private static StatusCode retryConfiguration(
            Supplier<StatusCode> function, int deviceId, String deviceType) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < MAX_RETRIES; ++i) {
            status = function.get();
            if (status.isOK()) {
                return status;
            }
            try {
                Thread.sleep(20); // Small delay between retries
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        if (!status.isOK()) {
            String errorMsg =
                    String.format(
                            "Failed to configure %s (ID %d) after %d attempts. Error: %s",
                            deviceType, deviceId, MAX_RETRIES, status.toString());
            DriverStation.reportError(errorMsg, true);
        }

        return status;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return retryConfiguration(
                () -> motor.getConfigurator().apply(config), motor.getDeviceID(), "TalonFX");
    }

    public static StatusCode applyConfiguration(TalonFX motor, CurrentLimitsConfigs config) {
        return retryConfiguration(
                () -> motor.getConfigurator().apply(config), motor.getDeviceID(), "TalonFX");
    }

    public static StatusCode applyConfiguration(TalonFX motor, Slot0Configs config) {
        return retryConfiguration(
                () -> motor.getConfigurator().apply(config), motor.getDeviceID(), "TalonFX");
    }

    public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
        return retryConfiguration(
                () -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID(), "CANcoder");
    }

    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return retryConfiguration(
                () -> motor.getConfigurator().refresh(config), motor.getDeviceID(), "TalonFX");
    }
}
