package org.frc5687.robot.commands.drive;

import org.frc5687.robot.commands.OutliersCommand;
import org.frc5687.robot.subsystems.drive.CTREDriveIO;
import org.frc5687.robot.subsystems.drive.DriveSubsystem;

public class zeroIMU extends OutliersCommand{
    DriveSubsystem _drive;

    public zeroIMU(DriveSubsystem drive){

        _drive = drive;
    }

    @Override
    protected void execute(double timestamp) {
        _drive.resetHeading();
    }
}
