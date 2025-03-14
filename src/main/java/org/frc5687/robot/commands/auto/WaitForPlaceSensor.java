package org.frc5687.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5687.robot.RobotContainer;
import org.frc5687.robot.subsystems.coralarm.CoralArmSubsystem;

public class WaitForPlaceSensor extends Command {
    private CoralArmSubsystem _arm;

    public WaitForPlaceSensor(RobotContainer container) {
        _arm = container.getCoral();
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return _arm.isPlaceCoralPlaced();
    }
}
