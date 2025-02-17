package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristStuff.WristDirectControlSubsystem;

public class WristDirectControlCommand extends Command {

  private final WristDirectControlSubsystem sub;

  private final DoubleSupplier motorInput;

  public WristDirectControlCommand(WristDirectControlSubsystem _sub, DoubleSupplier _control) {
    sub = _sub;

    motorInput = _control;

    addRequirements(sub);
  }


  @Override
  public void initialize() {

  }


  @Override
  public void execute() {
    double speed = motorInput.getAsDouble();
    speed = MathUtil.applyDeadband(speed, 0.001);
    speed *= 0.1;
    speed = Math.max(speed, -0.1);
    speed = Math.min(speed, 0.1);
    sub.runPercentage(speed);
  }


  @Override
  public void end(boolean interrupted) {

  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
