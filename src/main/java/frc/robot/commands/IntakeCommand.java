package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.INTAKE_POSITION;
import frc.robot.subsystems.Intake.Intake;

public class IntakeCommand extends Command {

  private final Intake intakeSub;
  private final INTAKE_POSITION position;


  public IntakeCommand(Intake intakeSub, INTAKE_POSITION position) {
    this.intakeSub = intakeSub;
    this.position = position;

    addRequirements(intakeSub);
  }

  @Override
  public void initialize() {
    switch (position) {
      case INTAKE:
        intakeSub.goToPosition(Constants.IntakeConstants.ExtensionMechanism.extendedPosition);
        break;
      case HALF_STOW:
        intakeSub.goToPosition(Constants.IntakeConstants.ExtensionMechanism.half_stowPosition);
        break;
      case STOW:
        intakeSub.goToPosition(Constants.IntakeConstants.ExtensionMechanism.retractedPosition);
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
