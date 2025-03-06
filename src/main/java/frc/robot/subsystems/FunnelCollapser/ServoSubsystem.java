package frc.robot.subsystems.FunnelCollapser;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoSubsystem extends SubsystemBase {

    private final Servo servo1;
    private final Servo servo2;

    public ServoSubsystem(int servo1ID, int servo2ID) {
        servo1 = new Servo(servo1ID);
        servo2 = new Servo(servo2ID);
    }

    public void GoToAngle(double angleDegrees) {
        angleDegrees = Math.max(angleDegrees, 0);
        angleDegrees = Math.min(angleDegrees, 180);

        servo1.setAngle(angleDegrees);
        servo2.setAngle(angleDegrees);
    }
}
