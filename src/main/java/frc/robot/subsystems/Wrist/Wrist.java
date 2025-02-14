package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Wrist extends SubsystemBase {

    private static double kDt = 0.02;


  private final Joystick m_joystick = new Joystick(1);

  private final SparkFlex wristMax;
  private final AbsoluteEncoder wristEncoder;

  // Note: These gains are fake, and will have to be tuned for your robot.
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 1.5);

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =
         new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    public Wrist(int wristMotorID) {

        wristMax = new SparkFlex(wristMotorID, MotorType.kBrushless);
        SparkFlexConfig wristConfig = new SparkFlexConfig();
        //  Configure
        wristMax.configure(wristConfig, null, null);


        wristEncoder = wristMax.getAbsoluteEncoder();
        
    }
}

