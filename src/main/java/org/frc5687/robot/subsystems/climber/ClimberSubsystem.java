package org.frc5687.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class ClimberSubsystem extends OutliersSubsystem<ClimberInputs, ClimberOutputs> {

    private final ProfiledPIDController _pidController;

    public ClimberSubsystem(RobotContainer container, ClimberIO io) {
        super(container, io, new ClimberInputs(), new ClimberOutputs());

        _pidController =
                new ProfiledPIDController(
                        Constants.Climber.kP,
                        0.0, // kI
                        Constants.Climber.kD,
                        new TrapezoidProfile.Constraints(
                                Constants.Climber.FAST_VELOCITY_RAD_PER_SEC, Constants.Climber.MAX_ACCELERATION));

        _pidController.reset(_inputs.motorAngleRads);
    }

    @Override
    protected void processInputs() {}

    @Override
    protected void periodic(ClimberInputs inputs, ClimberOutputs outputs) {
        double pidOutput = _pidController.calculate(inputs.motorAngleRads);
        double setpoint = _pidController.getGoal().position;
        double velocitySetpoint = _pidController.getSetpoint().velocity;

        outputs.motorSetpointRads = setpoint;
        outputs.motorVelocityRadPerSec = velocitySetpoint;
        outputs.climberVoltage = pidOutput;
    }

    public boolean isAtDesiredAngle() {
        return _pidController.atGoal();
    }

    public void toggleClimberSetpoint() {
        if (_outputs.motorSetpointRads != Constants.Climber.CLIMBER_UP_RADS) {
            _outputs.servoSetpoint = 0.0;
            _pidController.setConstraints(
                    new TrapezoidProfile.Constraints(
                            Constants.Climber.SLOW_VELOCITY_RAD_PER_SEC, Constants.Climber.MAX_ACCELERATION));
            _pidController.setGoal(Constants.Climber.CLIMBER_UP_RADS);
            _outputs.motorVelocityRadPerSec = Constants.Climber.SLOW_VELOCITY_RAD_PER_SEC;
            _outputs.motorSetpointRads = Constants.Climber.CLIMBER_UP_RADS;
        } else {
            _outputs.servoSetpoint = 0.6;
            _pidController.setConstraints(
                    new TrapezoidProfile.Constraints(
                            Constants.Climber.FAST_VELOCITY_RAD_PER_SEC, Constants.Climber.MAX_ACCELERATION));
            _pidController.setGoal(Constants.Climber.CLIMBER_DOWN_RADS);
            _outputs.motorVelocityRadPerSec = Constants.Climber.FAST_VELOCITY_RAD_PER_SEC;
            _outputs.motorSetpointRads = Constants.Climber.CLIMBER_DOWN_RADS;
        }
    }

    public boolean isSensorTriggered() {
        return _inputs.sensor;
    }
}
