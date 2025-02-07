package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Limelight;

public class AutoFactory {

    public static NetworkTableEntry targetPoseTable;
    
    public static Command GeneratePathToAprilTagRelativePose(Pose2d relativePose, Limelight limelight){

        Pose2d targetPoseRobotSpace;

        // NOTICE: Continue here

        Command output = Commands.sequence(
            
        );

        return null;
    }
    
}