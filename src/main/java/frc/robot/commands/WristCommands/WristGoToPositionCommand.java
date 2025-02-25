package frc.robot.commands.WristCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.WristAngle;
import frc.robot.subsystems.WristStuff.Wrist;

public class WristGoToPositionCommand extends Command {

  private final Wrist wristSub;
  private final WristAngle wristAngle;


  public WristGoToPositionCommand(Wrist _wrist, WristAngle _wristAngle) {
    wristSub = _wrist;
    wristAngle = _wristAngle;

    addRequirements(wristSub);
  }

  @Override
  public void initialize() {
    switch (wristAngle) {
      case PASSTHROUGH:
        wristSub.setTargetAngle(Constants.WristConstants.passthroughPositionRad);
        break;
      case L1:
        wristSub.setTargetAngle(Constants.WristConstants.L1PositionRad);
        break;
      case L2L3:
        wristSub.setTargetAngle(Constants.WristConstants.L2L3PositionRad);
        break;
      case L4:
        wristSub.setTargetAngle(Constants.WristConstants.L4PositionRad);
        break;
      default:
        end(true);
        break;
    }
  }

  @Override
  public void execute() {
    wristSub.updateTowardsCurrentGoal();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return wristSub.hasReachedGoal();
  }
}
