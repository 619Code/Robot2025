package frc.robot.subsystems.WristStuff;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.WristConstants.WristAngleRad;


//  NOTICE: At some point, try making these functions not have a default.
//          They should be able to be non-default, but I'm in the middle of something right now and won't test it
public interface WristIO {

    @AutoLog
    public static class WristIOInputs {
        public double wristPosition = 0.0;
        public double wristVelocity = 0.0;
        public double wristSetpointPositionRad = 0.0;
        public double wristGoalPositionRad = WristAngleRad.FREEHANG.positionRad;
    }


    public void updateInputs(WristIOInputs inputs);

    public void setVoltage(double _angleRad);

}
