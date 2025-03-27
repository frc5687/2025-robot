package org.frc5687.robot.commands.elevator;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;
import org.frc5687.robot.subsystems.elevator.ElevatorSubsystem;
import org.frc5687.robot.subsystems.superstructure.SuperstructureState;
import org.frc5687.robot.util.ReefAlignmentHelpers;

public class GoToAlgaeHeight extends OutliersCommand {
    private final ElevatorSubsystem _elevator;
    private final DriveSubsystem _drive;
    private SuperstructureState _algaeHeight;

    public GoToAlgaeHeight(ElevatorSubsystem elevator, DriveSubsystem drive) {
        _elevator = elevator;
        _drive = drive;
        addRequirements(_elevator);
    }

    @Override
    public void initialize() {
        _algaeHeight = ReefAlignmentHelpers.calculateAlgaeHeight(() -> _drive.getPose()).get();
        _elevator.setDesiredHeight(_algaeHeight.getElevator().get());
    }

    @Override
    public void execute(double timestamp) {}

    @Override
    public boolean isFinished() {
        return _elevator.isAtDesiredPosition();
        // return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
