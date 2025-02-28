package org.frc5687.robot.commands.algae;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class EjectAlgae extends OutliersCommand {

    private final AlgaeArmSubsystem _algae;
    private double _timeLastSeen;
    private static final double EXTRA_TIME = 0.200;

    public EjectAlgae(AlgaeArmSubsystem algae) {
        _algae = algae;
        addRequirements(_algae);
        _timeLastSeen = Timer.getFPGATimestamp();
    }

    @Override
    public void initialize() {}

    @Override
    protected void execute(double timestamp) {
        if (_algae.isAlgaeDetected()) _timeLastSeen = Timer.getFPGATimestamp();
        // if (_algae.isSafeToEject()) {
            _algae.setWheelMotorVoltage(-12);
        // } else {
        //     _algae.setWheelMotorVoltage(0);
        // }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - _timeLastSeen > EXTRA_TIME;
    }
}
