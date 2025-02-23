package org.frc5687.robot.subsystems.climber;

import org.frc5687.robot.Constants;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.OutliersSubsystem;

public class ClimberSubsystem extends OutliersSubsystem<ClimberInputs, ClimberOutputs> {

    public ClimberSubsystem(RobotContainer container, ClimberIO io) {
        super(container, io, new ClimberInputs(), new ClimberOutputs());
    }

    @Override
    protected void processInputs() {}

    @Override
    protected void periodic(ClimberInputs inputs, ClimberOutputs outputs) {}

    public boolean isAtDesiredAngle() {
        return Math.abs(_inputs.motorAngleRads - _outputs.motorSetpointRads) < 1.0;
    }

    public void setDesiredAngle(double angle) {
        _outputs.motorSetpointRads = angle;
    }

    public void toggleClimberSetpoint() {
        if (_outputs.motorSetpointRads != Constants.Climber.CLIMBER_UP_RADS) {
            _outputs.servoSetpoint = 0.0;
            _outputs.motorSetpointRads = Constants.Climber.CLIMBER_UP_RADS;
        } else {
            _outputs.servoSetpoint = 0.52;
            _outputs.motorSetpointRads = Constants.Climber.CLIMBER_DOWN_RADS;
        }
    }
}
