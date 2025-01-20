package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
//import frc.robot.helpers.Crashboard;
import frc.robot.subsystems.drive.ManipulatorSubsystem;

public class ShootCommand extends Command {

    private ManipulatorSubsystem subsystem;
    private boolean hasReachedVelocity = false;

    private double outtakeSpeed;
    private double intakeSpeed;
    private int  RPMsRequiredForOuttake;
    
    public ShootCommand(ManipulatorSubsystem subsystem) {
        this.subsystem = subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (OurRobotState.currentArmPosition == ArmPosEnum.AMP) {

 //           this.outtakeSpeed = Constants.ManipulatorConstants.outtakeSpeedAmp;
            this.outtakeSpeed = Constants.ManipulatorConstants.outtakeSpeedSpeakerVoltage;
            this.intakeSpeed = Constants.ManipulatorConstants.intakeSpeedWhenOuttaking;
            this.RPMsRequiredForOuttake = Constants.ManipulatorConstants.ampShooterVelocityToReachBeforeFeedingNote;

        } else if (OurRobotState.currentArmPosition == ArmPosEnum.SPEAKER) {

            //this.outtakeSpeed = Constants.ManipulatorConstants.outtakeSpeedSpeaker;
            this.outtakeSpeed = Constants.ManipulatorConstants.outtakeSpeedSpeakerVoltage;
            this.intakeSpeed = Constants.ManipulatorConstants.intakeSpeedWhenOuttaking;
            this.RPMsRequiredForOuttake = Constants.ManipulatorConstants.speakerShooterVelocityToReachBeforeFeedingNote;

        } else if (OurRobotState.currentArmPosition == ArmPosEnum.LONG_SHOT) {
            this.outtakeSpeed = Constants.ManipulatorConstants.outtakeSpeedSpeakerVoltage;
            this.intakeSpeed = Constants.ManipulatorConstants.intakeSpeedWhenOuttaking;
            this.RPMsRequiredForOuttake = Constants.ManipulatorConstants.passerShooterVelocityToReachBeforeFeedingNote;
        } else {
            // do nothing, no shooting!
                // Shooter, no shooting!
            this.outtakeSpeed = 0;
            this.intakeSpeed = 0;
            this.RPMsRequiredForOuttake = 0;
        }

        //subsystem.spinShooterVoltage(this.outtakeSpeed); // test value, make sure to change once we g
        subsystem.setShooterSpeedByRPM(RPMsRequiredForOuttake);
    }

    @Override
    public void execute() {
        
         //Crashboard.toDashboard("shooter flywheel RPMS: ", subsystem.GetShooterVelocity(), "shooter");

        if(subsystem.GetShooterVelocity()   >= this.RPMsRequiredForOuttake * 0.90){

            hasReachedVelocity = true;

        }

        if(hasReachedVelocity){

            subsystem.spintake(this.intakeSpeed);

        }
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        hasReachedVelocity = false;
        subsystem.stopShooter();
        subsystem.stopIntake();
    }
}