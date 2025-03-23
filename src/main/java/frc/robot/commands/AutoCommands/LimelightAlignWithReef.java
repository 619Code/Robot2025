// package frc.robot.commands.AutoCommands;


// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.PathConstraints;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.util.FieldCoordinatePose2d;

// public class LimelightAlignWithReef extends Command {

//     private Drive driveSubsystem;

//     private Command littleCommand;

//     private final boolean right;

//     private final double horizontalOffsetMeters = Units.inchesToMeters(5);

//     private boolean cantFindTag = false;


//     public LimelightAlignWithReef(Drive _driveSubsystem, boolean _right){
//         driveSubsystem = _driveSubsystem;
//         right = _right;

//         addRequirements(driveSubsystem);
//     }

//     @Override
//     public void initialize(){

//         double horizontalOffset = horizontalOffsetMeters;
//         horizontalOffset *= right ? 1.0 : -1.0;

//         Pose2d poseRelativeToAprilTag = new Pose2d(
//             Units.inchesToMeters(15) + 0.34,
//             horizontalOffset,
//             new Rotation2d(Math.PI)
//         );

//         FieldCoordinatePose2d tagPose = driveSubsystem.getViewedAprilTagPoseFieldSpace();

//         if(tagPose == null){
//             cantFindTag = true;
//             return;
//         }

//         Pose2d desiredEndPose = tagPose.pose.
//             transformBy(new Transform2d(new Pose2d(), poseRelativeToAprilTag));

//         StructPublisher<Pose2d> pub = NetworkTableInstance.getDefault().getStructTopic(
//         "Pathfinding destination", Pose2d.struct
//         ).publish();

//         pub.set(desiredEndPose);

//         PathConstraints constraints = new PathConstraints(1.5, 2, Math.PI * 2.0, Math.PI * 2.0);

//         littleCommand = AutoBuilder.pathfindToPose(desiredEndPose, constraints);


//         littleCommand.initialize();

//     }


//     @Override
//     public void execute() {
//         if(cantFindTag) {
//             return;
//         }

//         littleCommand.execute();

//     }


//     @Override
//     public void end(boolean isInterrupted) {
//         if(cantFindTag) {
//             return;
//         }

//         littleCommand.end(isInterrupted);

//     }

//     @Override
//     public boolean isFinished() {
//         if(cantFindTag) {
//             return true;
//         }

//         return littleCommand.isFinished();
//     }
// }
