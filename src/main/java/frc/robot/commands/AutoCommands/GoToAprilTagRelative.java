package frc.robot.commands.AutoCommands;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;

public class GoToAprilTagRelative extends Command {

    private Drive driveSubsystem;

    private Command littleCommand;

    private Pose2d poseRelativeToAprilTag;

    public GoToAprilTagRelative(Pose2d _poseRelativeToAprilTag, Drive _driveSubsystem){
        driveSubsystem = _driveSubsystem;
        poseRelativeToAprilTag = _poseRelativeToAprilTag;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize(){

        double[] limelightData = LimelightHelpers.getTargetPose_RobotSpace("limelight");

        Pose2d aprilTagRelativePose = limelightCoordsToWPICoordsPose2d(limelightData);
        Pose2d drivetrainPos = driveSubsystem.getPose();
        drivetrainPos = drivetrainPos.plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));


        Pose2d apriltagFieldCoords = drivetrainPos.transformBy(new Transform2d(new Pose2d(), aprilTagRelativePose));

        StructPublisher<Pose2d> otherPub = NetworkTableInstance.getDefault().getStructTopic(
            "April tag field coords", Pose2d.struct
            ).publish();

            otherPub.set(apriltagFieldCoords);


        Pose2d desiredEndPose = apriltagFieldCoords.transformBy(new Transform2d(new Pose2d(), poseRelativeToAprilTag));

        StructPublisher<Pose2d> pub = NetworkTableInstance.getDefault().getStructTopic(
        "Pathfinding destination", Pose2d.struct
        ).publish();

        pub.set(desiredEndPose);

        PathConstraints constraints = new PathConstraints(0.5, 1, Math.PI * 2.0, Math.PI * 2.0);

        littleCommand = AutoBuilder.pathfindToPose(desiredEndPose, constraints);


        littleCommand.initialize();

    }

    private Pose2d limelightCoordsToWPICoordsPose2d(double[] limelightData) {
        return new Pose2d(limelightData[2], -limelightData[0], new Rotation2d(Math.toRadians(-limelightData[4] + 180)));
    }


    @Override
    public void execute() {
        littleCommand.execute();
    }

    @Override
    public void end(boolean isInterrupted) {
        littleCommand.end(isInterrupted);
    }

    @Override
    public boolean isFinished() {
        return littleCommand.isFinished();
    }
}
