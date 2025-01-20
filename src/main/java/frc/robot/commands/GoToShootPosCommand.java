package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.subsystems.drive.HingeSubsystem;

public class GoToShootPosCommand extends Command {
    private HingeSubsystem subsystem;

    public GoToShootPosCommand(Subsystem subsystem) {
        this.subsystem = (HingeSubsystem) subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if (OurRobotState.isClimbing) {
            subsystem.setGoal(70);
        } else {
            subsystem.setGoal(Constants.HingeConstants.kShootingAngle);
        }
        subsystem.enable();
        //System.out.println("I am initialize");
        OurRobotState.currentArmPosition = ArmPosEnum.SPEAKER;
    }

    @Override
    public void execute() {
        subsystem.enable();
        //System.out.println("I am execute");
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.disable();
    }

    @Override
    public boolean isFinished() {
        //return subsystem.isAtPosition(Constants.HingeConstants.kShootingAngle, 0 /*temp deadzone*/);
        //return subsystem.getController().atGoal();
        return false;
    }
}
