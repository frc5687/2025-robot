package org.frc5687.robot.commands.algae;

import edu.wpi.first.wpilibj.Timer;
import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.algaearm.AlgaeArmSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorState;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.util.TunableDouble;

public class EjectAlgae extends OutliersCommand {

    private final AlgaeArmSubsystem _algae;
    private final ElevatorSubsystem _elevator;
    private double _timeLastSeen;
    private static final double EXTRA_TIME = 0.200;

    private static final TunableDouble netVoltage = new TunableDouble("EjectAlgae", "netvoltage", -12);

    public EjectAlgae(AlgaeArmSubsystem algae, ElevatorSubsystem elevator) {
        _algae = algae;
        _elevator = elevator;
        addRequirements(_algae);
        _timeLastSeen = Timer.getFPGATimestamp();
    }

    @Override
    public void initialize() {}

    @Override
    protected void execute(double timestamp) {
        if (_algae.isAlgaeDetected()) _timeLastSeen = Timer.getFPGATimestamp();
        // if (_algae.isSafeToEject()) {
        if (_elevator.getElevatorHeight() < ElevatorState.LOW_ALGAE_GRAB.getHeight()) {
            _algae.setWheelMotorVoltage(-2);
        } else {
            _algae.setWheelMotorVoltage(netVoltage.get());
        }
        // } else {
        //     _algae.setWheelMotorVoltage(0);
        // }
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - _timeLastSeen > EXTRA_TIME;
    }
}
