package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class FieldCoordinatePose2d {

    public Pose2d pose;

    public FieldCoordinatePose2d(Pose2d _pose){
        pose = _pose;
    }

    public RelativeCoordinatePose2d toRelativeCoords(Pose2d objectToBeRelativeTo){
        return new RelativeCoordinatePose2d(
            new Pose2d().transformBy(new Transform2d(objectToBeRelativeTo, pose))
        );
    }

    public RelativeCoordinatePose2d toRelativeCoords(FieldCoordinatePose2d objectToBeRelativeTo){
        return new RelativeCoordinatePose2d(
            new Pose2d().transformBy(new Transform2d(objectToBeRelativeTo.pose, pose))
        );
    }
}
