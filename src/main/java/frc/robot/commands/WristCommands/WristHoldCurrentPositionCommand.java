package frc.robot.commands.WristCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristStuff.Wrist;

public class WristHoldCurrentPositionCommand extends Command {

  private final Wrist wristSub;


  public WristHoldCurrentPositionCommand(Wrist _wrist) {
    wristSub = _wrist;

    addRequirements(wristSub);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    wristSub.updateTowardsCurrentGoal();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
