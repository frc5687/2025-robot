package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;

public class IdleElevator extends OutliersCommand {

    private final ElevatorSubsystem _elevator;
    private boolean swap = false;
    private boolean lock = false;

    public IdleElevator(ElevatorSubsystem elevator) {
        _elevator = elevator;
        addRequirements(_elevator);
    }

    @Override
    protected void execute(double timestamp) {
        //         if ((int) timestamp % 2 == 0 && !lock) {
        //     _elevator.setDesiredState(swap ? ElevatorState.STOWED : ElevatorState.L3_CORAL_PLACING);
        //     swap = !swap;
        //     lock = true;
        // } else {
        //     lock = (int) timestamp % 2 == 0;
        // }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}
