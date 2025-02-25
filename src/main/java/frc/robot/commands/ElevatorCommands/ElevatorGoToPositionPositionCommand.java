package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.Elevator;

public class ElevatorGoToPositionPositionCommand extends Command {

  private final Elevator elevatorSubsystem;

  private final double targetAngle;

  public ElevatorGoToPositionPositionCommand(Elevator _sub, double _angleRad) {
    elevatorSubsystem = _sub;

    targetAngle = _angleRad;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.setTargetPosition(targetAngle);
  }

  @Override
  public void execute() {
    elevatorSubsystem.updateTowardsCurrentGoal();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.hasReachedGoal();
  }
}
