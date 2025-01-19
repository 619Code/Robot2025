package frc.robot.subsystems.drive;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  NetworkTableEntry tx;
  // NetworkTableEntry ty;

  public Limelight() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    //     ty = table.getEntry("ty");

  }

  public double GetLimelightOffset() {
    return tx.getDouble(0.0);
  }
}
