package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Limelight;

public class CenterOnAprilTagCommand extends Command {

  private final Limelight limelightSubsystem;
  private final Drive swerveSubsystem;

  // private DoubleEntry kpRotationNetworkEntry;
  // private DoubleEntry kiRotationNetworkEntry;
  // private DoubleEntry kdRotationNetworkEntry;

  // private DoubleEntry kpRot2NetworkEntry;
  // private DoubleEntry kiRot2NetworkEntry;
  // private DoubleEntry kdRot2NetworkEntry;

  // private DoubleEntry kpMovementNetworkEntry;
  // private DoubleEntry kiMovementNetworkEntry;
  // private DoubleEntry kdMovementNetworkEntry;

  private NetworkTableEntry targetPoseTable;

  //  IntegerEntry filterTapsNetworkEntry;
  int filterTaps = 3;

  private final double speed = 2.0;

  private final PIDController rotationPid;
  private final PIDController YmovementPid;
  private final PIDController XmovementPid;
  private final PIDController rot2Pid;

  //    private final DoublePublisher anglePub;
  private final DoublePublisher desiredSpeedPub;

  private LinearFilter rotationFilter;
  private LinearFilter xMovementFilter;
  private LinearFilter yMovementFilter;
  private LinearFilter rot2Filter;

  double[] targetPosData;

  Timer generalTimer;

  private enum CenteringStates {
    MOVING_AND_ROTATING,
    IDLE
  }

  private CenteringStates currentState;

  private double desiredZOffset = 1.5;
  private double desiredXOffset = 0;
  private double desiredRotationOffset = 0;

  private DoublePublisher rotationErrorPub;
  private DoublePublisher xPositionErrorPub;
  private DoublePublisher yPositionErrorPub;

  public CenterOnAprilTagCommand(Drive _swerveSubsystem, Limelight _limelightSub) {
    limelightSubsystem = _limelightSub;
    swerveSubsystem = _swerveSubsystem;

    rotationPid = new PIDController(0.07, 0, 0.0016);
    YmovementPid = new PIDController(2, 0, 0.1);
    XmovementPid = new PIDController(2, 0, 0.1);
    // YmovementPid = new PIDController(0.6, 0, 0);
    // XmovementPid = new PIDController(0.6, 0, 0);
    rot2Pid = new PIDController(0.07, 0, 0.0016);

    //      anglePub = NetworkTableInstance.getDefault().getDoubleTopic("LimelightAngle").publish();
    desiredSpeedPub =
        NetworkTableInstance.getDefault().getDoubleTopic("LimelightDesiredSpeed").publish();

    rotationErrorPub =
        NetworkTableInstance.getDefault().getDoubleTopic("LimelightRotationError").publish();
    xPositionErrorPub = NetworkTableInstance.getDefault().getDoubleTopic("xPosError").publish();
    yPositionErrorPub = NetworkTableInstance.getDefault().getDoubleTopic("yPosError").publish();

    rotationErrorPub.set(0.0);
    xPositionErrorPub.set(0.0);
    yPositionErrorPub.set(0.0);

    // kpRotationNetworkEntry  =
    // NetworkTableInstance.getDefault().getDoubleTopic("kpR").getEntry(0.0);
    // kiRotationNetworkEntry  =
    // NetworkTableInstance.getDefault().getDoubleTopic("kiR").getEntry(0.0);
    // kdRotationNetworkEntry  =
    // NetworkTableInstance.getDefault().getDoubleTopic("kdR").getEntry(0.0);

    // kpRot2NetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kpR2").getEntry(0.0);
    // kiRot2NetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kiR2").getEntry(0.0);
    // kdRot2NetworkEntry  = NetworkTableInstance.getDefault().getDoubleTopic("kdR2").getEntry(0.0);

    // kpMovementNetworkEntry  =
    // NetworkTableInstance.getDefault().getDoubleTopic("kpM").getEntry(0.0);
    // kiMovementNetworkEntry  =
    // NetworkTableInstance.getDefault().getDoubleTopic("kiM").getEntry(0.0);
    // kdMovementNetworkEntry  =
    // NetworkTableInstance.getDefault().getDoubleTopic("kdM").getEntry(0.0);

    targetPoseTable =
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace");
    //     filterTapsNetworkEntry =
    // NetworkTableInstance.getDefault().getIntegerTopic("fTaps").getEntry(0);

    // kpRotationNetworkEntry.set(0.0);
    // kiRotationNetworkEntry.set(0.0);
    // kdRotationNetworkEntry.set(0.0);

    // kpRot2NetworkEntry.set(0.0);
    // kiRot2NetworkEntry.set(0.0);
    // kdRot2NetworkEntry.set(0.0);

    // kpMovementNetworkEntry.set(0.0);
    // kiMovementNetworkEntry.set(0.0);
    // kdMovementNetworkEntry.set(0.0);

    //     filterTapsNetworkEntry.set(filterTaps);

    rotationFilter = LinearFilter.movingAverage(filterTaps);
    xMovementFilter = LinearFilter.movingAverage(3);
    yMovementFilter = LinearFilter.movingAverage(3);
    rot2Filter = LinearFilter.movingAverage(3);

    generalTimer = new Timer();

    addRequirements(_limelightSub);
    addRequirements(_swerveSubsystem);
  }

  @Override
  public void initialize() {
    SwitchToState(CenteringStates.MOVING_AND_ROTATING);
  }

  @Override
  public void execute() {
    //  limelightSubsystem.SendRandomDataToShuffleboard();

    RetrieveDataFromTables();

    double xSpeedMpS = 0;
    double ySpeedMpS = 0;
    double rotationSpeedRpS = 0;

    switch (currentState) {
      case MOVING_AND_ROTATING:
        rotationSpeedRpS = AimPerpindicularToAprilTag();
        //     rotationSpeedRpS = AimTowardsAprilTag();

        double[] speeds = MovingToPoint();

        xSpeedMpS = speeds[0];
        ySpeedMpS = speeds[1];

        CheckStateMachineMovingAndRottingStage();
        break;
      case IDLE:
        break;
      default:
        break;
    }

    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            -xSpeedMpS,
            -ySpeedMpS,
            rotationSpeedRpS,
            Rotation2d.fromDegrees(targetPosData[4])); // from Field
    swerveSubsystem.runVelocity(speeds);
  }

  private void CheckStateMachineMovingAndRottingStage() {

    boolean isInCorrectPosition = Math.abs(desiredRotationOffset - targetPosData[4]) < 4;
    isInCorrectPosition =
        isInCorrectPosition && (Math.abs(desiredXOffset - targetPosData[0]) < 0.0254);
    isInCorrectPosition =
        isInCorrectPosition && (Math.abs(desiredZOffset + targetPosData[2]) < 0.0254);

    if (isInCorrectPosition) {
      //       Crashboard.toDashboard("Is in correct position:", generalTimer.get(), "Limelight");
      if (generalTimer.get() > 0.1) {
        SwitchToState(CenteringStates.IDLE);
      }
    } else {
      generalTimer.reset();
    }
  }

  private void SwitchToState(CenteringStates _state) {
    currentState = _state;
    generalTimer.reset();
    generalTimer.start();
  }

  private double[] MovingToPoint() {

    double targetXOffset = targetPosData[0];

    targetXOffset = desiredXOffset - targetXOffset;

    xPositionErrorPub.set(targetXOffset);

    double targetYOffset = targetPosData[2];
    //  Make it go to desiredZOffset meters away
    targetYOffset = desiredZOffset + targetYOffset;

    yPositionErrorPub.set(targetYOffset);

    // Crashboard.toDashboard("Tx: ", targetPosData[0], "Limelight");
    // Crashboard.toDashboard("Ty: ", targetPosData[1], "Limelight");
    // Crashboard.toDashboard("Tz: ", targetPosData[2], "Limelight");
    // Crashboard.toDashboard("Pitch: ", targetPosData[3], "Limelight");
    // Crashboard.toDashboard("Yaw: ", targetPosData[4], "Limelight");
    // Crashboard.toDashboard("Roll: ", targetPosData[5], "Limelight");

    double ySpeed = -YmovementPid.calculate(targetXOffset, 0);
    double xSpeed = -XmovementPid.calculate(targetYOffset, 0);

    // ySpeed = Math.min(2, ySpeed) * angleMultiplier;
    // xSpeed = Math.min(2, xSpeed) * angleMultiplier;

    ySpeed = Math.min(speed, ySpeed);
    xSpeed = Math.min(speed, xSpeed);
    ySpeed = Math.max(-speed, ySpeed);
    xSpeed = Math.max(-speed, xSpeed);

    return new double[] {xSpeed, ySpeed};
    //   return new double[]{0, 0};
  }

  private double AimPerpindicularToAprilTag() {

    double angleDifference = targetPosData[4];

    angleDifference = desiredRotationOffset - angleDifference;

    rotationErrorPub.set(angleDifference);

    double speedRadiansPerSecond = -rot2Pid.calculate(angleDifference, 0);
    //    speedRadiansPerSecond *= 0.9;

    return speedRadiansPerSecond;
  }

  private double AimTowardsAprilTag() {

    double angleDifference = limelightSubsystem.GetLimelightOffset();
    angleDifference = rotationFilter.calculate(angleDifference);

    double speedRadiansPerSecond = -rotationPid.calculate(angleDifference, 0);

    return speedRadiansPerSecond;
  }

  private void RetrieveDataFromTables() {
    targetPosData = targetPoseTable.getDoubleArray(new double[6]);
    targetPosData[0] = xMovementFilter.calculate(targetPosData[0]);
    targetPosData[2] = yMovementFilter.calculate(targetPosData[2]);
    targetPosData[4] = rot2Filter.calculate(targetPosData[4]);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
