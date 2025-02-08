package frc.robot.subsystems.not_drive;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ledSubsystem extends SubsystemBase {

  private CANdle candle;
  private Animation currentAnimation = null;

  public ledSubsystem() {
    this.candle = new CANdle(Constants.LEDConstants.CANdleID);

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.v5Enabled = true;
    configAll.statusLedOffWhenActive = false;
    configAll.disableWhenLOS = false;
    configAll.stripType = LEDStripType.RGB;
    configAll.brightnessScalar = .7;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    this.candle.configAllSettings(configAll, 100);
  }

  public void setColor(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  public void setColor(int r, int g, int b, int w, int startlid, int count) {
    candle.setLEDs(r, g, b, w, startlid, count);
  }

  public void TurnOffLEDs() {
    candle.setLEDs(0, 0, 0);
  }
}
