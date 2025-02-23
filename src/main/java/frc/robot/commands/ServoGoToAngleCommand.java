package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelCollapser.ServoSubsystem;

public class ServoGoToAngleCommand extends Command {

    private final ServoSubsystem subsystem;
    private final double angle;

    public ServoGoToAngleCommand(ServoSubsystem _sub, double _angle){
        subsystem = _sub;

        angle = _angle;

        addRequirements(subsystem);
    }


    @Override
    public void initialize(){
        subsystem.GoToAngle(angle);
    }
}
