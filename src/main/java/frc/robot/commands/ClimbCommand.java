package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.CLIMB_POSITION;
import frc.robot.subsystems.Climb.Climb;

public class ClimbCommand extends Command{

  private final Climb climbSub;
  private final CLIMB_POSITION position;


  public ClimbCommand(Climb climbSub, CLIMB_POSITION position) {
    this.climbSub = climbSub;
    this.position = position;

    addRequirements(climbSub);
  }

  @Override
  public void initialize() {
    switch (position) {
      case OUT:
        climbSub.goToPosition(Constants.ClimbConstants.climbOutPosition);
        break;
      case IN:
        climbSub.goToPosition(Constants.ClimbConstants.climbInPosition);
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
    return true;
  }
}
