package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.ManipulatorSubsystem;

public class IntakeCommand extends Command{

    private ManipulatorSubsystem subsystem;
    
    public IntakeCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spintake(Constants.ManipulatorConstants.intakeSpeed); // test value plz change. NO! I WON'T!
    }

    @Override
    public boolean isFinished() {
        System.out.println(subsystem.intakeTrigged());
        return subsystem.intakeTrigged(); 
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopIntake();
    }
}
