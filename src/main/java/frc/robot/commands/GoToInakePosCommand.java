package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.helpers.ArmPosEnum;
import frc.robot.subsystems.drive.HingeSubsystem;

public class GoToInakePosCommand extends Command {
    private HingeSubsystem subsystem;

    public GoToInakePosCommand(Subsystem subsystem) {
        this.subsystem = (HingeSubsystem) subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.setGoal(Constants.HingeConstants.kIntakeAngle);
        subsystem.enable();
        OurRobotState.currentArmPosition = ArmPosEnum.INTAKE;
        OurRobotState.isClimbing = false;

    }

    @Override
    public void end(boolean interrupted) {
        subsystem.disable();
    }

    @Override
    public boolean isFinished() {
        //return subsystem.isAtPosition(Constants.HingeConstants.kIntakeAngle, 0 /*temp deadzone*/);
        //return subsystem.getController().atGoal();
        return false;
    }
}
