package frc.robot.commands.WristCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.WristConstants.WristAngleRad;
import frc.robot.subsystems.WristStuff.Wrist;

public class WristGoToPositionCommand extends Command {

  private final Wrist wristSub;
  private final WristAngleRad wristAngle;


  public WristGoToPositionCommand(Wrist _wrist, WristAngleRad _wristAngle) {
    wristSub = _wrist;
    wristAngle = _wristAngle;

    addRequirements(wristSub);
  }

  @Override
  public void initialize() {
    wristSub.setTargetAngle(wristAngle.positionRad);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return wristSub.hasReachedGoal();
  }
}
