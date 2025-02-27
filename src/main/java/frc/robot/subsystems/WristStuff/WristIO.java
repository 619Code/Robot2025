package frc.robot.subsystems.WristStuff;

import org.littletonrobotics.junction.AutoLog;


//  NOTICE: At some point, try making these functions not have a default.
//          They should be able to be non-default, but I'm in the middle of something right now and won't test it
public interface WristIO {

    @AutoLog
    public static class WristIOInputs {
        public double wristPosition = 0.0;
        public double wristSetpointPosition = 0.0;
    }


    public void updateInputs(WristIOInputs inputs);

    public void ioPeriodic();


    public void setTargetAngle(double _angleRad);

    //  NOTICE: Eventually this should definitely be changed.
    //  I see no way this would work with the replay feature as it is now
    public boolean hasReachedGoal();
}
