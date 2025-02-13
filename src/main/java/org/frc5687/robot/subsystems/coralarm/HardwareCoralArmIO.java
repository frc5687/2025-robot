package org.frc5687.robot.subsystems.coralarm;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotMap;
import org.frc5687.robot.util.TunableDouble;
import org.frc5687.robot.util.sensors.ProximitySensor;

public class HardwareCoralArmIO implements CoralArmIO {
    private final CANcoder _cancoder;
    private final VictorSP _pivotMotor;
    private final VictorSP _wheelMotor;
    private final ProfiledPIDController _controller;
    private final ProximitySensor _coralDetectionSensor;
    private final Debouncer _debouncer;

    private final StatusSignal<Angle> _absoluteAngle;

    private final TunableDouble kP = new TunableDouble("Coral", "kP", Constants.CoralArm.kP);
    private final TunableDouble kD = new TunableDouble("Coral", "kD", Constants.CoralArm.kD);

    public HardwareCoralArmIO() {
        _cancoder = new CANcoder(RobotMap.CAN.CANCODER.CORAL_ENCODER, "CANivore");

        _pivotMotor = new VictorSP(RobotMap.PWM.CORAL_PIVOT_MOTOR);
        _wheelMotor = new VictorSP(RobotMap.PWM.CORAL_WHEEL_MOTOR);
        _coralDetectionSensor = new ProximitySensor(RobotMap.DIO.CORAL_SENSOR);
        _debouncer = new Debouncer(.1, Debouncer.DebounceType.kRising);
        TrapezoidProfile.Constraints constraints =
                new TrapezoidProfile.Constraints(
                        Constants.CoralArm.MAX_VELOCITY_RAD_PER_SEC,
                        Constants.CoralArm.MAX_ACCELERATION_RAD_PER_SEC_SQUARED);

        _controller = new ProfiledPIDController(kP.get(), Constants.CoralArm.kI, kD.get(), constraints);

        _controller.setTolerance(0.01);
        _pivotMotor.setInverted(Constants.CoralArm.PIVOT_MOTOR_INVERTED);
        _wheelMotor.setInverted(Constants.CoralArm.WHEEL_MOTOR_INVERTED);

        configureCancoder();
        _absoluteAngle = _cancoder.getAbsolutePosition();
        _controller.reset(getAngleRads());
    }

    private void calculateShortestPath(double currentAngle) {
        if (currentAngle < Units.degreesToRadians(90) || currentAngle >= Units.degreesToRadians(270)) {
            _controller.enableContinuousInput(0, 2.0 * Math.PI);
        } else {
            _controller.disableContinuousInput();
        }
    }

    private double processSafeAngle(double desiredAngle) {
        return MathUtil.clamp(desiredAngle, Constants.CoralArm.MIN_ANGLE, Constants.CoralArm.MAX_ANGLE);
    }

    private double calculateFeedForward(double angle) {
        return ((Constants.CoralArm.ARM_LENGTH / 2.0)
                        * (Constants.CoralArm.GEARBOX.rOhms * Constants.CoralArm.ARM_MASS * 9.81)
                        / (Constants.CoralArm.GEAR_RATIO * Constants.CoralArm.GEARBOX.KtNMPerAmp))
                * Math.cos(angle);
    }

    @Override
    public void updateInputs(CoralInputs inputs) {
        StatusSignal.refreshAll(_absoluteAngle);
        inputs.angleRads = getAngleRads();
        inputs.isCoralDetected = _debouncer.calculate(_coralDetectionSensor.get());
        inputs.motorCurrent = 0.0;
    }

    @Override
    public void writeOutputs(CoralOutputs outputs) {
        double currentAngle = getAngleRads();
        double safeAngle = processSafeAngle(outputs.desiredAngleRad);
        calculateShortestPath(currentAngle);

        _controller.setP(kP.get());
        _controller.setD(kD.get());

        _controller.setGoal(safeAngle);

        double pidOutput = _controller.calculate(currentAngle);
        double ffOutput = calculateFeedForward(currentAngle);

        double totalVoltage = MathUtil.clamp(pidOutput + ffOutput, -12.0, 12.0);

        outputs.voltageCommand = totalVoltage;
        outputs.controllerOutput = pidOutput;
        outputs.voltageFeedForward = ffOutput;

        _pivotMotor.setVoltage(totalVoltage);
        _wheelMotor.setVoltage(outputs.wheelVoltageCommand);
    }

    private double getAngleRads() {
        return _absoluteAngle.getValueAsDouble() * 2.0 * Math.PI;
    }

    private void configureCancoder() {
        var _cancoderConfigs = new CANcoderConfiguration();
        _cancoderConfigs.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(1));
        _cancoderConfigs.MagnetSensor.MagnetOffset = Constants.CoralArm.ENCODER_OFFSET;
        _cancoderConfigs.MagnetSensor.SensorDirection =
                SensorDirectionValue.Clockwise_Positive; // FIXME
        _cancoder.getConfigurator().apply(_cancoderConfigs);
        // CTREUtil.applyConfiguration(_cancoder, _cancoderConfigs);
    }
}
