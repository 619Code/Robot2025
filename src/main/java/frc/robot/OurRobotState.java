package frc.robot;

import frc.robot.helpers.ArmPosEnum;

public class OurRobotState {
    public static boolean ArmInitialized = false;
    public static ArmPosEnum currentArmPosition = ArmPosEnum.SPEAKER;
    public static boolean isClimbing = false;
    public static boolean isEnabled = false;
    public static boolean hasNote = false;
}