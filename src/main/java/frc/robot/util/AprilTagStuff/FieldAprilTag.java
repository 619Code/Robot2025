package frc.robot.util.AprilTagStuff;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N4;

public class FieldAprilTag {

    public AprilTagInfo info;

    public Pose3d pose;

    /**
    *  April tag id indexing start at 1. The first april tag has an id of 1, not 0.
    */
    public FieldAprilTag(int _id){
        info = AprilTagDataLoader.aprilTags[_id - 1];

        Matrix<N4, N4> matrix = new Matrix<N4, N4>(N4.instance, N4.instance, info.transform);

        pose = new Pose3d(matrix);

    }


    //  Yay
}
