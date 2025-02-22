package frc.robot.commands.AutoCommands;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.FieldCoordinatePose2d;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.RelativeCoordinatePose2d;

public class AutoFactoryGen2 {

    public static Command PathfindRelativeToAprilTag(Pose2d poseRelativeToAprilTag, Drive driveSubsystem){


        double[] limelightData = LimelightHelpers.getTargetPose_RobotSpace("limelight");

        RelativeCoordinatePose2d aprilTagRelativePose = new RelativeCoordinatePose2d(limelightCoordsToWPICoordsPose2d(limelightData));
        FieldCoordinatePose2d drivetrainPos = new FieldCoordinatePose2d(driveSubsystem.getPose());
        drivetrainPos.pose = drivetrainPos.pose.plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));


        FieldCoordinatePose2d apriltagFieldCoords = aprilTagRelativePose.toFieldSpace(drivetrainPos);

        // Publishing
        StructPublisher<Pose2d> otherPub = NetworkTableInstance.getDefault().getStructTopic(
            "April tag field coords", Pose2d.struct
            ).publish();

        otherPub.set(apriltagFieldCoords.pose);


        FieldCoordinatePose2d desiredEndPose = new FieldCoordinatePose2d(
            apriltagFieldCoords.pose.transformBy(new Transform2d(new Pose2d(), poseRelativeToAprilTag))
        );

        // Publishing
        StructPublisher<Pose2d> pub = NetworkTableInstance.getDefault().getStructTopic(
        "Pathfinding destination", Pose2d.struct
        ).publish();

        pub.set(desiredEndPose.pose);

        PathConstraints constraints = new PathConstraints(0.5, 1, Math.PI * 2.0, Math.PI * 2.0);

        Command littleCommand = AutoBuilder.pathfindToPose(desiredEndPose.pose, constraints);

        return littleCommand;

    }


    private static Pose2d limelightCoordsToWPICoordsPose2d(double[] limelightData) {
        return new Pose2d(limelightData[2], -limelightData[0], new Rotation2d(Math.toRadians(-limelightData[4] + 180)));
    }
}
