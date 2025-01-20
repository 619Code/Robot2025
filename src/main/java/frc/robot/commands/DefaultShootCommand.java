package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ManipulatorSubsystem;

public class DefaultShootCommand extends Command {

    private ManipulatorSubsystem subsystem;
    
    public DefaultShootCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(subsystem.intakeTrigged()){

            subsystem.setShooterSpeedByRPM(Constants.ManipulatorConstants.shooterIdleRPM);

        }else{

            subsystem.stopShooter();

        }
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
    }
}