package org.frc5687.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.frc5687.robot.Constants;
import org.frc5687.robot.util.CTREUtil;
import org.frc5687.robot.util.TunableDouble;

public class HardwareElevatorIO implements ElevatorIO {
    private final TalonFX _eastMotor;
    private final TalonFX _westMotor;

    private final StatusSignal<AngularAcceleration> _westAcceleration;
    private final StatusSignal<AngularVelocity> _westVelocity;
    private final StatusSignal<Angle> _westPosition;
    private final StatusSignal<AngularAcceleration> _eastAcceleration;
    private final StatusSignal<AngularVelocity> _eastVelocity;
    private final StatusSignal<Angle> _eastPosition;
    private final StatusSignal<Current> _eastCurrent;
    private final StatusSignal<Current> _westCurrent;

    private final MotionMagicExpoTorqueCurrentFOC _westPositionTorqueRequest;
    private final MotionMagicExpoTorqueCurrentFOC _eastPositionTorqueRequest;

    private final VoltageOut _westVoltageRequest;
    private final VoltageOut _eastVoltageRequest;

    // private final LaserCan _laserCan;

    private double _platformVelocity = 0.0;
    // private boolean _zeroed = false;
    private boolean _safetyTripped = false;

    private TunableDouble _positionDifferenceThreshold =
            new TunableDouble("Elevator", "PositionDifferenceThreshold", 0.05);

    public HardwareElevatorIO(int eastMotorId, int westMotorId, int laserCanId) {
        // _laserCan = new LaserCan(laserCanId);

        _eastMotor = new TalonFX(eastMotorId, Constants.Elevator.CANBUS);
        _westMotor = new TalonFX(westMotorId, Constants.Elevator.CANBUS);

        _westMotor.setPosition(0);
        _eastMotor.setPosition(0);
        _westVelocity = _westMotor.getVelocity();
        _westPosition = _westMotor.getPosition();
        _westCurrent = _westMotor.getSupplyCurrent();
        _eastVelocity = _eastMotor.getVelocity();
        _eastPosition = _eastMotor.getPosition();
        _eastCurrent = _eastMotor.getSupplyCurrent();

        _westPositionTorqueRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);
        _eastPositionTorqueRequest = new MotionMagicExpoTorqueCurrentFOC(0).withSlot(0);

        _westVoltageRequest = new VoltageOut(0);
        _eastVoltageRequest = new VoltageOut(0);

        _westAcceleration = _westMotor.getAcceleration();
        _eastAcceleration = _eastMotor.getAcceleration();

        setSignalFrequency();
        setControlFrequency();

        configureMotor(_eastMotor, Constants.Elevator.EAST_INVERTED);
        configureMotor(_westMotor, Constants.Elevator.WEST_INVERTED);
        _westMotor.optimizeBusUtilization();
        _eastMotor.optimizeBusUtilization();
    }

    private void setControlFrequency() {
        _westPositionTorqueRequest.UpdateFreqHz = 1000;
        _eastPositionTorqueRequest.UpdateFreqHz = 1000;
    }

    private void setSignalFrequency() {
        _westPosition.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _westVelocity.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _westAcceleration.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _westCurrent.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);

        _eastPosition.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _eastVelocity.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _eastAcceleration.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
        _eastCurrent.setUpdateFrequency(1.0 / Constants.UPDATE_PERIOD);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusSignal.refreshAll(
                _westAcceleration,
                _westVelocity,
                _westPosition,
                _westCurrent,
                _eastAcceleration,
                _eastVelocity,
                _eastPosition,
                _eastCurrent);

        double eastPosition =
                Units.rotationsToRadians(_eastPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;

        double westPosition =
                Units.rotationsToRadians(_westPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;

        double eastVelocity =
                _eastVelocity.getValueAsDouble()
                        * (2 * Math.PI * Constants.Elevator.DRUM_RADIUS)
                        / Constants.Elevator.GEAR_RATIO;

        double westVelocity =
                _westVelocity.getValueAsDouble()
                        * (2 * Math.PI * Constants.Elevator.DRUM_RADIUS)
                        / Constants.Elevator.GEAR_RATIO;

        inputs.eastPositionMeters = eastPosition;
        inputs.westPositionMeters = westPosition;
        inputs.eastVelocityMPS = eastVelocity;
        inputs.westVelocityMPS = westVelocity;

        double positionDifference = Math.abs(eastPosition - westPosition);
        double velocityDifference = Math.abs(eastVelocity - westVelocity);

        inputs.eastWestPositionDifference = positionDifference;
        inputs.eastWestVelocityDifference = velocityDifference;

        inputs.positionDifferenceSafetyThreshold = _positionDifferenceThreshold.get();

        if (
        /* _zeroed &&  */ positionDifference > _positionDifferenceThreshold.get()) {
            _safetyTripped = true;
            inputs.safetyStatus =
                    String.format(
                            "SAFETY TRIPPED: Position difference of %.3fm exceeds threshold", positionDifference);
            System.err.println(inputs.safetyStatus);
        }

        inputs.heightPositionMeters = (eastPosition + westPosition) / 2.0;

        _platformVelocity = (eastVelocity + westVelocity) / 2.0;

        inputs.platformAcceleration =
                (_eastAcceleration.getValueAsDouble() + _westAcceleration.getValueAsDouble())
                        / 2.0
                        * (2 * Math.PI * Constants.Elevator.DRUM_RADIUS)
                        / Constants.Elevator.GEAR_RATIO;

        inputs.firstStageVelocityMPS = _platformVelocity / 2.0;
        inputs.platformMotorCurrents =
                new double[] {
                    _eastCurrent.getValueAsDouble(), _westCurrent.getValueAsDouble(),
                };

        inputs.isDisabled = _safetyTripped;
        // inputs.zeroed = _zeroed;

        if (!_safetyTripped) {
            inputs.safetyStatus = "OK";
        }

        // LaserCan.Measurement measurement = _laserCan.getMeasurement();

        // if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
        // {
        //     inputs.laserSensorElevatorHeightMeters = measurement.distance_mm / 1000.0;

        //     if (!_zeroed && !_safetyTripped) {
        //         double laserMotorMeters = inputs.laserSensorElevatorHeightMeters - 0.020;
        //         double rotations =
        //                 laserMotorMeters
        //                         / Constants.Elevator.DRUM_RADIUS
        //                         * Constants.Elevator.GEAR_RATIO
        //                         / (2 * Math.PI);

        //         _eastMotor.setPosition(rotations);
        //         _westMotor.setPosition(rotations);
        //         _zeroed = true;
        //         inputs.zeroed = true;
        //         inputs.safetyStatus = "Zeroed successfully";
        //         System.out.println("Elevator zeroed successfully at height: " + laserMotorMeters +
        // "m");
        //     }
        // } else {
        //     inputs.laserSensorElevatorHeightMeters = -1;
        // }
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        // if (!_zeroed) {
        //     System.err.println("ERROR: ELEVATOR HAS NOT BEEN ZEROED, NO CONTROL COMMANDS WILL BE
        // SENT");
        //     return;
        // }

        if (_safetyTripped) {
            System.err.println("ERROR: ELEVATOR SAFETY TRIPPED - MOTORS DISABLED UNTIL RESET");
            // Stop motors
            _eastMotor.setControl(new VoltageOut(0));
            _westMotor.setControl(new VoltageOut(0));
            return;
        }

        double eastRotations =
                Units.radiansToRotations(outputs.desiredHeight / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;
        double westRotations =
                Units.radiansToRotations(outputs.desiredHeight / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;

        switch (outputs.controlMode) {
            case VOLTAGE:
                _eastMotor.setControl(_eastVoltageRequest.withOutput(outputs.voltageCommandEast));
                _westMotor.setControl(_westVoltageRequest.withOutput(outputs.voltageCommandWest));
                break;
            case POSITION:
                _eastMotor.setControl(_eastPositionTorqueRequest.withPosition(eastRotations));
                _westMotor.setControl(_westPositionTorqueRequest.withPosition(westRotations));
                break;
            default:
                break;
        }
    }

    private void configureMotor(TalonFX motor, boolean isInverted) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Elevator.MOTION_MAGIC_EXPO_KA;
        config.MotionMagic.MotionMagicExpo_kV = Constants.Elevator.MOTION_MAGIC_EXPO_KV;

        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;
        config.Slot0.kG = Constants.Elevator.kG;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        CTREUtil.applyConfiguration(motor, config);
    }

    public void setPID(double kP, double kI, double kD, double kV, double kS, double kA, double kG) {
        Slot0Configs config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = kV;
        config.kA = kA;
        config.kS = kS;
        config.kG = kG;

        CTREUtil.applyConfiguration(_eastMotor, config);
        CTREUtil.applyConfiguration(_westMotor, config);
    }

    @Override
    public void resetSafety() {
        if (_safetyTripped) {
            System.out.println("Resetting elevator safety");
            _safetyTripped = false;
        }
    }
}
