package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds.ledSubsystem;

public class LedAnimationCommand extends Command {
  private ledSubsystem subsystem;
  private boolean isEndGame = false;

  public LedAnimationCommand(ledSubsystem subsystem) {
    this.subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initialize() {
    // Initialize LED behavior here
  }

  @Override
  public void execute() {
    double matchTime = DriverStation.getMatchTime();

    if (matchTime <= 30 && matchTime != -1) {
      isEndGame = true;
      subsystem.setColor(255, 0, 0); // Red color during Endgame
    } else {
      isEndGame = false;
      subsystem.setColor(0, 0, 255); // Blue color during normal play
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // Optionally turn off LEDs when the command ends
    subsystem.TurnOffLEDs();
  }
}
// Need to look at this so see if correct
