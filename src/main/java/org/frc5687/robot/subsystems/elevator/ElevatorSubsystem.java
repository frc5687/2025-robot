

package org.frc5687.robot.subsystems.elevator;

import org.frc5687.robot.subsystems.OutliersSubsystem;

public class ElevatorSubsystem extends OutliersSubsystem<ElevatorInputs, ElevatorOutputs> {
     
    public ElevatorSubsystem(ElevatorIO io){
        super(io, new ElevatorInputs(), new ElevatorOutputs());

     }

    @Override
    protected void processInputs() {
        
    }

    @Override
    protected void periodic(ElevatorInputs inputs, ElevatorOutputs outputs) {
        
    }

    public void setPositionMeters(double positionMeters){
        _outputs.desiredElevatorPositionMeters = positionMeters;
    
    }
 

    public void setVoltage(double voltage){
        _outputs.elevatorVoltage = voltage;
 
    } 
}
/* 
 * class init
 * create private motors (through motor id's and configs in constants.java)
 * 
 * create getters/setters (public function that can set and get things (angles, speeds))
 * 
 * create commands in classes/new files (require the subsytems that you are using and call its methods)
 * 
*/