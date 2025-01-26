package org.frc5687.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.frc5687.robot.Constants;

public class HardwareElevatorIO implements ElevatorIO {
    private static final double MAX_CORRECTION = 0.05;

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

    private final MotionMagicVoltage _northWestPositionRequest;
    private final MotionMagicVoltage _northEastPositionRequest;
    private final MotionMagicVoltage _southWestPositionRequest;

    private final PIDController _pitchController;
    private final PIDController _rollController;

    private final VoltageOut _voltageRequest = new VoltageOut(0).withEnableFOC(true);

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

        _northWestPositionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _northEastPositionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);
        _southWestPositionRequest = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        _imuPitch = _imu.getPitch();
        _imuRoll = _imu.getRoll();

        configureMotor(_northEastElevatorMotor, Constants.Elevator.NORTH_EAST_INVERTED);
        configureMotor(_northWestElevatorMotor, Constants.Elevator.NORTH_WEST_INVERTED);
        configureMotor(_southWestElevatorMotor, Constants.Elevator.SOUTH_EAST_INVERTED);

        _northWestElevatorMotor.setPosition(0);
        _northEastElevatorMotor.setPosition(0);
        _southWestElevatorMotor.setPosition(0);

        _pitchController = new PIDController(0.01, 0.0, 0.0);
        _rollController = new PIDController(0.01, 0.0, 0.0);

        _pitchController.setSetpoint(0.0);
        _rollController.setSetpoint(0.0);

        _pitchController.setTolerance(0.1);
        _rollController.setTolerance(0.1);
    }

    public double[] calculateHeightCorrections(
            double baseHeight, double currentPitch, double currentRoll) {
        double pitchCorrection = _pitchController.calculate(currentPitch);
        double rollCorrection = _rollController.calculate(currentRoll);

        pitchCorrection = MathUtil.clamp(pitchCorrection, -MAX_CORRECTION, MAX_CORRECTION);
        rollCorrection = MathUtil.clamp(rollCorrection, -MAX_CORRECTION, MAX_CORRECTION);

        double northWest = baseHeight - pitchCorrection - rollCorrection;
        double northEast = baseHeight - pitchCorrection + rollCorrection;
        double southWest = baseHeight + pitchCorrection - rollCorrection;

        return new double[] {northWest, northEast, southWest};
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
                        / Constants.Elevator.GEAR_RATIO;
        inputs.stageNorthEastPositionMeters =
                Units.rotationsToRadians(_northEastPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;
        inputs.stageSouthWestPositionMeters =
                Units.rotationsToRadians(_southWestPosition.getValueAsDouble())
                        * Constants.Elevator.DRUM_RADIUS
                        / Constants.Elevator.GEAR_RATIO;

        inputs.platformMotorCurrents =
                new double[] {
                    _northWestElevatorMotor.getSupplyCurrent().getValueAsDouble(),
                    _northEastElevatorMotor.getSupplyCurrent().getValueAsDouble(),
                    _southWestElevatorMotor.getSupplyCurrent().getValueAsDouble()
                };
    }

    @Override
    public void writeOutputs(ElevatorOutputs outputs) {
        double desiredHeight = outputs.desiredStageHeight;
        // double desiredRotations = Units.radiansToRotations(desiredHeight /
        // Constants.Elevator.DRUM_RADIUS) * Constants.Elevator.GEAR_RATIO;

        // System.out.println(_northEastElevatorMotor.getClosedLoopOutput());
        double[] correctedHeights =
                calculateHeightCorrections(
                        desiredHeight, _imuPitch.getValueAsDouble(), _imuRoll.getValueAsDouble());

        double nwRotations =
                Units.radiansToRotations(correctedHeights[0] / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;
        double neRotations =
                Units.radiansToRotations(correctedHeights[1] / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;
        double swRotations =
                Units.radiansToRotations(correctedHeights[2] / Constants.Elevator.DRUM_RADIUS)
                        * Constants.Elevator.GEAR_RATIO;

        _northWestElevatorMotor.setControl(_northWestPositionRequest.withPosition(nwRotations));
        _northEastElevatorMotor.setControl(_northEastPositionRequest.withPosition(neRotations));
        _southWestElevatorMotor.setControl(_southWestPositionRequest.withPosition(swRotations));
    }

    private void configureMotor(TalonFX motor, boolean isInverted) {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        double metersToRotations = 1.0 / (2.0 * Math.PI * Constants.Elevator.DRUM_RADIUS);

        config.MotionMagic.MotionMagicCruiseVelocity =
                Constants.Elevator.MAX_VELOCITY_MPS * metersToRotations;
        config.MotionMagic.MotionMagicAcceleration =
                Constants.Elevator.MAX_ACCELERATION_MPSS * metersToRotations;
        config.MotionMagic.MotionMagicJerk = Constants.Elevator.MAX_JERK_MPSSS * metersToRotations;

        config.Slot0.kP = Constants.Elevator.kP;
        config.Slot0.kI = Constants.Elevator.kI;
        config.Slot0.kD = Constants.Elevator.kD;
        config.Slot0.kS = Constants.Elevator.kS;
        config.Slot0.kV = Constants.Elevator.kV;
        config.Slot0.kA = Constants.Elevator.kA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Elevator.CURRENT_LIMIT;

        motor.getConfigurator().apply(config);
    }

    public boolean isLevel(double currentPitch, double currentRoll) {
        return _pitchController.atSetpoint() && _rollController.atSetpoint();
    }
}
