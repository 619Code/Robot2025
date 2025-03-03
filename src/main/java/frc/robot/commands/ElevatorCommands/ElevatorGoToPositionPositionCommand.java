package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorGoToPositionPositionCommand extends Command {

  private final Elevator elevatorSubsystem;

  private final ElevatorHeight targetHeight;

  public ElevatorGoToPositionPositionCommand(Elevator _sub, ElevatorHeight _targetHeight) {
    elevatorSubsystem = _sub;

    targetHeight = _targetHeight;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setTargetPosition(targetHeight);
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.hasReachedGoal();
  }
}
