package org.frc5687.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;

public class HardwareElevatorIO implements ElevatorIO {

    private final TalonFX _northWestElevatorMotor;
    private final TalonFX _northEastElevatorMotor;
    private final TalonFX _southWestElevatorMotor;
    private final Pigeon2 _imu;

    private final StatusSignal<AngularVelocity> _northWestVelocity;
    private final StatusSignal<Angle> _northWestPosition;
    private final StatusSignal<AngularVelocity> _northEastVelocity;
    private final StatusSignal<Angle> _northEastPosition;
    private final StatusSignal<AngularVelocity> _southWestVelocity;
    private final StatusSignal<Angle> _southWestPosition;

    private final StatusSignal<Angle> _imuPitch;
    private final StatusSignal<Angle> _imuRoll;

    private final PositionTorqueCurrentFOC _northWestPositionTorqueRequest;
    private final PositionTorqueCurrentFOC _northEastPositionTorqueRequest;
    private final PositionTorqueCurrentFOC _southWestPositionTorqueRequest;

    private final PIDController _pitchController;
    private final PIDController _rollController;

    private double _firstStageHeight = 0.0;
    private double _rollOffset = 0.0;
    private double _pitchOffset = 0.0;
    private double _platformVelocity = 0.0;

    public HardwareElevatorIO(
            int northWestMotorID, int northEastMotorID, int southEastMotorID, int imuId) {
        _northWestElevatorMotor = new TalonFX(northWestMotorID, Constants.Elevator.CANBUS);
        _northEastElevatorMotor = new TalonFX(northEastMotorID, Constants.Elevator.CANBUS);
        _southWestElevatorMotor = new TalonFX(southEastMotorID, Constants.Elevator.CANBUS);
        _imu = new Pigeon2(imuId, Constants.Elevator.CANBUS);

        _northWestVelocity = _northWestElevatorMotor.getVelocity();
        _northWestPosition = _northWestElevatorMotor.getPosition();
        _northEastVelocity = _northEastElevatorMotor.getVelocity();
        _northEastPosition = _northEastElevatorMotor.getPosition();
        _southWestVelocity = _southWestElevatorMotor.getVelocity();
        _southWestPosition = _southWestElevatorMotor.getPosition();

        _northWestPositionTorqueRequest = new PositionTorqueCurrentFOC(0).withSlot(1);
        _northEastPositionTorqueRequest = new PositionTorqueCurrentFOC(0).withSlot(1);
        _southWestPositionTorqueRequest = new PositionTorqueCurrentFOC(0).withSlot(1);

        _imuPitch = _imu.getPitch();
        _imuRoll = _imu.getRoll();

        setSignalFrequency();
        setControlFrequency();

        configureMotor(_northEastElevatorMotor, Constants.Elevator.NORTH_EAST_INVERTED, true);
        configureMotor(_northWestElevatorMotor, Constants.Elevator.NORTH_WEST_INVERTED, true);
        configureMotor(_southWestElevatorMotor, Constants.Elevator.SOUTH_EAST_INVERTED, false);

        _northWestElevatorMotor.setPosition(0);
        _northEastElevatorMotor.setPosition(0);
        _southWestElevatorMotor.setPosition(0);

        _pitchController =
                new PIDController(Constants.Elevator.PITCH_kP, 0.0, Constants.Elevator.PITCH_kD);
        _rollController =
                new PIDController(Constants.Elevator.ROLL_kP, 0.0, Constants.Elevator.ROLL_kD);

        _pitchController.setSetpoint(0.0);
        _rollController.setSetpoint(0.0);

        _pitchController.setTolerance(Units.degreesToRadians(0.1));
        _rollController.setTolerance(Units.degreesToRadians(0.1));

        _pitchOffset = _imuPitch.getValueAsDouble();
        _rollOffset = _imuRoll.getValueAsDouble();
    }

    private void setControlFrequency() {
        _northWestPositionTorqueRequest.UpdateFreqHz = 1000;
        _northEastPositionTorqueRequest.UpdateFreqHz = 1000;
        _southWestPositionTorqueRequest.UpdateFreqHz = 1000;
    }

    private void setSignalFrequency() {
        _northWestPosition.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _northWestVelocity.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);

        _southWestPosition.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _southWestVelocity.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);

        _northEastPosition.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _northEastVelocity.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);

        _imuPitch.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
        _imuRoll.setUpdateFrequency(1.0 / Constants.Elevator.PERIOD);
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        StatusSignal.waitForAll(
                .2,
                _northWestVelocity,
                _northWestPosition,
                _northEastVelocity,
                _northEastPosition,
                _southWestVelocity,
                _southWestPosition,
                _imuPitch,
                _imuRoll);

        inputs.stageNorthWestPositionMeters =
                Units.rotationsToRadians(_northWestPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO_NORTH;
        inputs.stageNorthEastPositionMeters =
                Units.rotationsToRadians(_northEastPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO_NORTH;
        inputs.stageSouthWestPositionMeters =
                Units.rotationsToRadians(_southWestPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO_SOUTH;

        inputs.firstStagePositionMeters =
                (inputs.stageNorthWestPositionMeters
                                + inputs.stageNorthEastPositionMeters
                                + inputs.stageSouthWestPositionMeters)
                        / 3.0;

        inputs.platformPitchRadians = Units.degreesToRadians(getPitch());
        inputs.platformRollRadians = Units.degreesToRadians(getRoll());

        inputs.driveTrainPitchRadians = Units.degreesToRadians(getPitch());
        inputs.driveTrainRollRadians = Units.degreesToRadians(getRoll());

        _platformVelocity =
                (_northEastVelocity.getValueAsDouble()
                                + _northWestVelocity.getValueAsDouble()
                                + _southWestVelocity.getValueAsDouble() / 3.0)
                        * (2 * Math.PI * Constants.Elevator.DRUM_RADIUS)
                        / Constants.Elevator.GEAR_RATIO_NORTH;
        inputs.platformVelocityMPS = _platformVelocity;
        inputs.platformMotorCurrents =
                new double[] {
                    _northWestElevatorMotor.getSupplyCurrent().getValueAsDouble(),
                    _northEastElevatorMotor.getSupplyCurrent().getValueAsDouble(),
                    _southWestElevatorMotor.getSupplyCurrent().getValueAsDouble()
                };

        _firstStageHeight = inputs.firstStagePositionMeters;
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double nwRotations =
                Units.radiansToRotations(outputs.northWestStageHeight / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO_NORTH;
        double neRotations =
                Units.radiansToRotations(outputs.northEastStageHeight / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO_NORTH;
        double swRotations =
                Units.radiansToRotations(outputs.southWestStageHeight / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO_SOUTH;

        outputs.usingPositionHolding = true;
        _northWestElevatorMotor.setControl(_northWestPositionTorqueRequest.withPosition(nwRotations));
        _northEastElevatorMotor.setControl(_northEastPositionTorqueRequest.withPosition(neRotations));
        _southWestElevatorMotor.setControl(_southWestPositionTorqueRequest.withPosition(swRotations));

        outputs.voltageCommandNorthEast =
                _northEastElevatorMotor.getClosedLoopOutput().getValueAsDouble();
        outputs.voltageCommandNorthWest =
                _northWestElevatorMotor.getClosedLoopOutput().getValueAsDouble();
        outputs.voltageCommandSouthWest =
                _southWestElevatorMotor.getClosedLoopOutput().getValueAsDouble();
    }

    private void configureMotor(TalonFX motor, boolean isInverted, boolean isNorth) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));
        double gearRatio =
                isNorth ? Constants.Elevator.GEAR_RATIO_NORTH : Constants.Elevator.GEAR_RATIO_SOUTH;
        double metersToRotations = (1.0 / (Constants.Elevator.DRUM_RADIUS)) * gearRatio;
        // Retain the configuration for motion magic/expo (only slot1 is used for position hold)
        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicExpo_kA = Constants.Elevator.MOTION_MAGIC_EXPO_KA;
        config.MotionMagic.MotionMagicExpo_kV = Constants.Elevator.MOTION_MAGIC_EXPO_KV;

        config.Slot1.kP = Constants.Elevator.HOLD_kP;
        config.Slot1.kI = Constants.Elevator.HOLD_kI;
        config.Slot1.kD = Constants.Elevator.HOLD_kD;
        config.Slot1.kS = Constants.Elevator.HOLD_kS;
        config.Slot1.kV = Constants.Elevator.HOLD_kV;
        config.Slot1.kA = Constants.Elevator.HOLD_kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        motor.getConfigurator().apply(config);
    }

    public boolean isLevel(double currentPitch, double currentRoll) {
        return _pitchController.atSetpoint() && _rollController.atSetpoint();
    }

    private double getPitch() {
        return _imuPitch.getValueAsDouble() - _pitchOffset;
    }

    private double getRoll() {
        return _imuRoll.getValueAsDouble() - _rollOffset;
    }
}
