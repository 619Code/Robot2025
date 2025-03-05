package frc.robot.util;

public class Help {
    public static double clamp(double val, double min, double max){
        return Math.min(Math.max(val, min), max);
    }
}
