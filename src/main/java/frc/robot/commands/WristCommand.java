package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.WristAngle;
import frc.robot.subsystems.WristStuff.Wrist;

public class WristCommand extends Command {

  private final Wrist wristSub;
  private final WristAngle johnson;


  public WristCommand(Wrist _wrist, WristAngle _johnson) {
    wristSub = _wrist;
    johnson = _johnson;

    addRequirements(wristSub);
  }

  @Override
  public void initialize() {
    switch (johnson) {
      case PASSTHROUGH:
        wristSub.goToPassthroughAngle();
        break;
      case L1:
        wristSub.goToL1Angle();
        break;
      case L2L3:
        wristSub.goToL2L3Angle();
        break;
      case L4:
        wristSub.goToL4Angle();
        break;
      default:
        end(true);
        break;
    }
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
