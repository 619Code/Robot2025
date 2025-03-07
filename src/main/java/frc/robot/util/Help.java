package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Help {
    public static double clamp(double val, double min, double max){
        return Math.min(Math.max(val, min), max);
    }


    public static Pose2d limelightCoordsToWPICoordsPose2d(double[] limelightData) {
        return new Pose2d(limelightData[2], -limelightData[0], new Rotation2d(Math.toRadians(-limelightData[4] + 180)));
    }
}
