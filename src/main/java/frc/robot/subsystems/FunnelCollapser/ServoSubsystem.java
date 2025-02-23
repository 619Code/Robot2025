package frc.robot.subsystems.FunnelCollapser;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {

    private final Servo servo;

    public ServoSubsystem(int _servoID){

        servo = new Servo(_servoID);

    }


    public void GoToAngle(double angleDegrees){

        angleDegrees = Math.max(angleDegrees, 0);
        angleDegrees = Math.min(angleDegrees, 180);

        servo.setAngle(angleDegrees);
    }
}
