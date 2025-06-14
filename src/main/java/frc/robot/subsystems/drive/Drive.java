// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.config.PIDConstants;
//import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.pathfinding.Pathfinding;
//import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Gyro.GyroIO;
import frc.robot.subsystems.drive.Gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.Gyro.GyroIONavX;
import frc.robot.subsystems.drive.Module.Module;
import frc.robot.subsystems.drive.Module.ModuleIOSim;
import frc.robot.subsystems.drive.Module.ModuleIOSpark;
import frc.robot.util.FieldCoordinatePose2d;
import frc.robot.util.Help;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.RelativeCoordinatePose2d;
import frc.robot.util.AprilTagStuff.AprilTagDataLoader;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);


  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.DriveConstants.moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();  // NOTICE: Try putting PI in these parenthesis to fix the autonomous problem.
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());


  public Drive() {
      if(Robot.isReal()){
        this.gyroIO = new GyroIONavX();
        modules[0] = new Module(
          new ModuleIOSpark(
                0,
                Constants.DriveConstants.frontLeftDriveMotorInverted,
                Constants.DriveConstants.frontLeftTurnMotorInverted,
                Constants.DriveConstants.frontLeftTurnEncoderInverted,
                Constants.DriveConstants.frontLeftDriveAbsoluteEncoderPort,
                Constants.DriveConstants.frontLeftDriveAbsoluteEncoderOffsetRots,
                Constants.DriveConstants.frontLeftEncoderPositiveDirection
                ), 0);
        modules[1] = new Module(
          new ModuleIOSpark(
            1,
            Constants.DriveConstants.frontRightDriveMotorInverted,
            Constants.DriveConstants.frontRightTurnMotorInverted,
            Constants.DriveConstants.frontRightTurnEncoderInverted,
            Constants.DriveConstants.frontRightDriveAbsoluteEncoderPort,
            Constants.DriveConstants.frontRightDriveAbsoluteEncoderOffsetRots,
            Constants.DriveConstants.frontRightEncoderPositiveDirection
          ), 1);
        modules[2] = new Module(new ModuleIOSpark(
            2,
            Constants.DriveConstants.backLeftDriveMotorInverted,
            Constants.DriveConstants.backLeftTurnMotorInverted,
            Constants.DriveConstants.backLeftTurnEncoderInverted,
            Constants.DriveConstants.backLeftDriveAbsoluteEncoderPort,
            Constants.DriveConstants.backLeftDriveAbsoluteEncoderOffsetRots,
            Constants.DriveConstants.backLeftEncoderPositiveDirection
          ), 2);
        modules[3] = new Module(new ModuleIOSpark(
            3,
            Constants.DriveConstants.backRightDriveMotorInverted,
            Constants.DriveConstants.backRightTurnMotorInverted,
            Constants.DriveConstants.backRightTurnEncoderInverted,
            Constants.DriveConstants.backRightDriveAbsoluteEncoderPort,
            Constants.DriveConstants.backRightDriveAbsoluteEncoderOffsetRots,
            Constants.DriveConstants.backRightEncoderPositiveDirection
          ), 3);
      }else{
        this.gyroIO = new GyroIO() {};
        modules[0] = new Module(new ModuleIOSim(), 0);
        modules[1] = new Module(new ModuleIOSim(), 1);
        modules[2] = new Module(new ModuleIOSim(), 2);
        modules[3] = new Module(new ModuleIOSim(), 3);
      }



    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();


    kinematics = new SwerveDriveKinematics(Constants.DriveConstants.moduleTranslations); //new SwerveDriveKinematics(Constants.DriveConstants.robotConfig.moduleLocations);


    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> runVelocity(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(2, 0, 0), // Translation PID constants
                    new PIDConstants(2, 0.0, 0.0) // Rotation PID constants
            ),
            Constants.DriveConstants.ppConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );




    Pathfinding.setPathfinder(new LocalADStarAK());

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          Logger.recordOutput("Odometry/TrajectoryCurrentPose", pose);
        });


    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data

    if(Constants.currentMode == Mode.REPLAY){
      //  There isnt a Logger.processInputs here because it was overwriting gyroInputs to be too correct
      gyroIO.updateInputs(gyroInputs);
    }else{
      gyroIO.updateInputs(gyroInputs);
      Logger.processInputs("Drive/Gyro", gyroInputs);
    }

    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);

      // Pose2d visionPose = getRobotVisionPose();
      // double confidence = getVisionConfidence();
      // Logger.recordOutput("Drive/VisionConfidence", confidence);
      // if(visionPose != null && confidence > 0.5){
      //  poseEstimator.addVisionMeasurement(visionPose, sampleTimestamps[i], Constants.DriveConstants.driveStandardDevs.times(1.0 / confidence));
      // }
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }


  private double getVisionConfidence(){

    double minDistance = 0.5;
    double maxDistance = 5.0;

    double output = ((distanceToTag - minDistance) / (maxDistance - minDistance));

    output = Math.max(output, 0);
    output = Math.min(output, 1);

    output = 1.0 - output;

    return output;

  }

  public FieldCoordinatePose2d getViewedAprilTagPoseFieldSpace(){

    int tagId = (int)LimelightHelpers.getFiducialID("limelight");

    Optional<Pose3d> tagPose3d = AprilTagDataLoader.field.getTagPose(tagId);

    if(tagPose3d.isEmpty()) return null;

    Pose2d actualTagCoordTemp = new Pose2d(tagPose3d.get().getX(), tagPose3d.get().getY(), tagPose3d.get().getRotation().toRotation2d());

    return new FieldCoordinatePose2d(actualTagCoordTemp);

  }

  double distanceToTag = 1.0;

  private Pose2d getRobotVisionPose(){
    double [] limelightData = LimelightHelpers.getBotPose_TargetSpace("limelight");

    FieldCoordinatePose2d actualTagCoord = getViewedAprilTagPoseFieldSpace();

    if(actualTagCoord == null){
      return null;
    }

    RelativeCoordinatePose2d aprilTagRelativePose = new RelativeCoordinatePose2d(Help.limelightCoordsToWPICoordsPose2d(limelightData));
    RelativeCoordinatePose2d robotPoseTagSpace = new RelativeCoordinatePose2d(new Pose2d(
      -aprilTagRelativePose.pose.getX(),
      -aprilTagRelativePose.pose.getY(),
      aprilTagRelativePose.pose.getRotation()
    ));

    distanceToTag = Math.sqrt((robotPoseTagSpace.pose.getX() * robotPoseTagSpace.pose.getX()) + (robotPoseTagSpace.pose.getY() * robotPoseTagSpace.pose.getY()));


    FieldCoordinatePose2d robotPoseField = robotPoseTagSpace.toFieldSpace(actualTagCoord);


    Logger.recordOutput("Limelight/RobotCalculatedPose", robotPoseField.pose);
    Logger.recordOutput("Limelight/RobotRelativeToAprilTag", robotPoseTagSpace.pose);

  return robotPoseField.pose;

  }


  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.kDt);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, Constants.DriveConstants.maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = Constants.DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    System.out.println("Resetting gyro angle: " + rawGyroRotation);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return Constants.DriveConstants.maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return Constants.DriveConstants.maxSpeedMetersPerSec
        / Constants.DriveConstants.driveBaseRadius
        * Constants.DriveConstants.angularVelocityMultiplier;
  }

  // public void SetModuleTurnMotorPD(double kp, double kd) {
  //   modules[0].setTurnMotorPID(kp, kd);
  //   modules[1].setTurnMotorPID(kp, kd);
  //   modules[2].setTurnMotorPID(kp, kd);
  //   modules[3].setTurnMotorPID(kp, kd);
  // }
}
