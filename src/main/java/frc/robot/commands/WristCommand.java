package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.WRIST_ANGLE;
import frc.robot.subsystems.WristStuff.Wrist;

public class WristCommand extends Command {

  private final Wrist wristSub;
  private final WRIST_ANGLE wristAngle;


  public WristCommand(Wrist _wrist, WRIST_ANGLE _wristAngle) {
    wristSub = _wrist;
    wristAngle = _wristAngle;

    addRequirements(wristSub);
  }

  @Override
  public void initialize() {
    switch (wristAngle) {
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
