package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class RelativeCoordinatePose2d {

    public Pose2d pose;

    public RelativeCoordinatePose2d(Pose2d _pose){
        pose = _pose;
    }

    public FieldCoordinatePose2d toFieldSpace(Pose2d objectWhichIAmRelativeTo){
        return new FieldCoordinatePose2d(
            objectWhichIAmRelativeTo.transformBy(new Transform2d(new Pose2d(), pose))
        );
    }

    public FieldCoordinatePose2d toFieldSpace(FieldCoordinatePose2d objectWhichIAmRelativeTo){
        return new FieldCoordinatePose2d(
            objectWhichIAmRelativeTo.pose.transformBy(new Transform2d(new Pose2d(), pose))
        );
    }
}
