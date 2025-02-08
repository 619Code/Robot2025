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

package frc.robot;

//import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.AutoCommands.GoToAprilTagRelative;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.Limelight;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Limelight limelight = new Limelight();

  // Controller
  //  private final CommandXboxController controller = new CommandXboxController(0);
  private final Joystick flightStick = new Joystick(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

 //   private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(
                    0,
                    Constants.DriveConstants.frontLeftDriveMotorInverted,
                    Constants.DriveConstants.frontLeftTurnMotorInverted,
                    Constants.DriveConstants.frontLeftTurnEncoderInverted,
                    Constants.DriveConstants.frontLeftDriveAbsoluteEncoderPort,
                    Constants.DriveConstants.frontLeftDriveAbsoluteEncoderOffsetDeg,
                    Constants.DriveConstants.frontLeftEncoderPositiveDirection), // FRONT LEFT
                new ModuleIOSpark(
                    1,
                    Constants.DriveConstants.frontRightDriveMotorInverted,
                    Constants.DriveConstants.frontRightTurnMotorInverted,
                    Constants.DriveConstants.frontRightTurnEncoderInverted,
                    Constants.DriveConstants.frontRightDriveAbsoluteEncoderPort,
                    Constants.DriveConstants.frontRightDriveAbsoluteEncoderOffsetDeg,
                    Constants.DriveConstants.frontRightEncoderPositiveDirection), // FRONT RIGHT
                new ModuleIOSpark(
                    2,
                    Constants.DriveConstants.backLeftDriveMotorInverted,
                    Constants.DriveConstants.backLeftTurnMotorInverted,
                    Constants.DriveConstants.backLeftTurnEncoderInverted,
                    Constants.DriveConstants.backLeftDriveAbsoluteEncoderPort,
                    Constants.DriveConstants.backLeftDriveAbsoluteEncoderOffsetDeg,
                    Constants.DriveConstants.backLeftEncoderPositiveDirection), // BACK LEFT
                new ModuleIOSpark(
                    3,
                    Constants.DriveConstants.backRightDriveMotorInverted,
                    Constants.DriveConstants.backRightTurnMotorInverted,
                    Constants.DriveConstants.backRightTurnEncoderInverted,
                    Constants.DriveConstants.backRightDriveAbsoluteEncoderPort,
                    Constants.DriveConstants.backRightDriveAbsoluteEncoderOffsetDeg,
                    Constants.DriveConstants.backRightEncoderPositiveDirection)); // BACK RIGHT
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    //  Publishing the never-flipped path (always relative to blue side)
    /* {
        try {
        List<Pose2d> examplePath = PathPlannerPath.fromPathFile("Lit path").getPathPoses();

        Pose2d[] posePath = new Pose2d[examplePath.size()];

        examplePath.toArray(posePath);

        StructArrayPublisher<Pose2d> pub =
            NetworkTableInstance.getDefault()
                .getStructArrayTopic("Lit path trajectory", Pose2d.struct)
                .publish();

        pub.set(posePath);

        } catch (Exception e) {
        System.out.println("Cry");
        }
    }*/

    // Set up auto routines
   autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    //Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));


    try{

    autoChooser.addOption("Basic ahh path",
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("Basic path")));

    }catch (Exception e){
        System.out.println("AHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH");
    }


    //  The below thing was a test for the pathplanning command
    /*{
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(3, 2, Rotation2d.fromDegrees(0));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                1.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);


        autoChooser.addOption("Pathfinding test",
        Commands.runOnce(() -> {
            drive.setPose(new Pose2d(2, 2, new Rotation2d(0)));
        }, drive).andThen(
        pathfindingCommand
        ));

    }*/

    autoChooser.addOption("Pathfinding with apriltag test",
        Commands.sequence(
            Commands.runOnce(() -> {
                drive.setPose(new Pose2d(1, 1, new Rotation2d(Math.PI)));
            }, drive),
            new GoToAprilTagRelative(new Pose2d(0.6, 0.1, new Rotation2d()), drive)
        )
    );




    AutoBuilder.buildAutoChooser();



    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -controller.getLeftY(),
    //         () -> -controller.getLeftX(),
    //         () -> -controller.getRightX()));

    //  Flight stick driving

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -flightStick.getRawAxis(1),
            () -> -flightStick.getRawAxis(0),
            () -> -flightStick.getRawAxis(2)));

    Trigger gyroResetButton = new JoystickButton(flightStick, 2);

    gyroResetButton.onTrue(
        Commands.runOnce(
                () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // controller.y().whileTrue((new CenterOnAprilTagCommand(drive, limelight)));

    // Trigger mainTrigger = new JoystickButton(flightStick, 1);
    // mainTrigger.whileTrue(new CenterOnAprilTagCommand(drive, limelight));

    // Trigger mainTrigger = new JoystickButton(flightStick, 1);
    // mainTrigger.whileTrue(new GoToAprilTagRelative(new Pose2d(1, 0, new Rotation2d()), drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

 //   return autoChooser.getSelected();



    return autoChooser.get();

    // try {
    //   //   Command followTrajectoryCommand =
    //   //       AutoBuilder.followPath(PathPlannerPath.fromPathFile("Lit path"))
    //   //           .andThen(
    //   //               Commands.run(
    //   //                   () -> {
    //   //                     System.out.println("FOLLOW PATH COMMAND IS FINISHED");
    //   //                   }));

    //   Command followTrajectoryCommand =
    //       AutoBuilder.followPath(PathPlannerPath.fromPathFile("Lit path"));

    //   return new LoggedCommand(followTrajectoryCommand);
    // } catch (Exception e) {
    //   System.out.println("AUTO FAILED");
    //   return null;
    // }
  }
}
