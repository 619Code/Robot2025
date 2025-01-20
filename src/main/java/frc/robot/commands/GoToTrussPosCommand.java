package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.subsystems.drive.HingeSubsystem;

public class GoToTrussPosCommand extends Command {
    private HingeSubsystem subsystem;

    public GoToTrussPosCommand(Subsystem s) {
        subsystem = (HingeSubsystem) s;
        addRequirements(s);
    }

    @Override
    public void initialize() {
        subsystem.setGoal(Constants.HingeConstants.kLongShotAngle);
        subsystem.enable();
        OurRobotState.currentArmPosition = ArmPosEnum.LONG_SHOT;
        OurRobotState.isClimbing = false;
    }

    

    @Override
    public boolean isFinished() {
        //return subsystem.getController().atGoal();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.disable();
    }
}
