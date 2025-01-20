package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.OurRobotState;
import frc.robot.subsystems.drive.HingeSubsystem;

public class HingeInitializeCommand extends Command {
    private HingeSubsystem subsystem;

    public HingeInitializeCommand(Subsystem subsystem) {
        this.subsystem = (HingeSubsystem) subsystem;

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        subsystem.spinge(-.07);
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stopHinge();
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }

    @Override
    public boolean isFinished() {
        if (subsystem.getAbsoluteDegrees() <= Constants.HingeConstants.kMinAngle+1) {
            subsystem.resetRelativeEncoders();
            OurRobotState.ArmInitialized = true;
            return true;
        }
        else {
            return false;
        }
    }
}
