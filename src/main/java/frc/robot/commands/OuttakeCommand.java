package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ManipulatorSubsystem;

public class OuttakeCommand extends Command { 

    private ManipulatorSubsystem mani;

    public OuttakeCommand(ManipulatorSubsystem pulator) {
        mani = pulator;
        addRequirements(mani);
    }

    @Override
    public void end(boolean interrupted) {
        mani.stopAll();
    }

    @Override
    public void execute() {
        mani.spinShooter(-Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote * 0.5);
        mani.spintake(-Constants.ManipulatorConstants.intakeSpeedWhenOuttaking * 0.5);
    }

    @Override
    public void initialize() {
        //
    }

    @Override
    public boolean isFinished() {
        return !mani.intakeTrigged();
    }
    
    
    
}
