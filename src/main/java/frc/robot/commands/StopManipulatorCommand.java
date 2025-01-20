package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.ManipulatorSubsystem;

public class StopManipulatorCommand extends Command{

    private ManipulatorSubsystem subsystem;
    
    public StopManipulatorCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.stopShooter();
        subsystem.stopIntake();
    }

    @Override
    public boolean isFinished(){
        //  Terminate command immediately after initialization
        return true;
    }
}