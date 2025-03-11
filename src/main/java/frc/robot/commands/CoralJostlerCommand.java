package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralRoller;

public class CoralJostlerCommand extends Command {
    CoralRoller roller;
    int counter = 0;
    int direction = 1;


     /** 
     * @param subsystem    The roller subsystem to use
     */
    public CoralJostlerCommand(CoralRoller subsystem) {
        roller = subsystem;
        addRequirements(roller);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      counter = 0;
    }

    @Override
    public void execute() {

        counter = counter + 1;
        if ((counter/5) % 5 == 4) {
            roller.runRollerRaw(-0.2);
        } else if ((counter/5) % 5 == 3){
            roller.runRollerRaw(0.1);
        }else{
            roller.runRollerRaw(0);
        }
    }

    @Override
    public void end(boolean interrupted){
        roller.runRollerRaw(0);
    
       
    }
}
