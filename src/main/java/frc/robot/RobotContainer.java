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
import frc.robot.commands.AutoCommands.AutoFactoryGen2;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Gyro.GyroIO;
import frc.robot.subsystems.drive.Gyro.GyroIONavX;
import frc.robot.subsystems.drive.Module.ModuleIO;
import frc.robot.subsystems.drive.Module.ModuleIOSim;
import frc.robot.subsystems.drive.Module.ModuleIOSpark;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;


public class RobotContainer {


    // Subsystems
    private final Drive drive;
    private final Intake intake;
    private final Wrist wrist;


    private final boolean driveEnabled = true, wristEnabled = true, intakeEnabled = true;


    // Controller
    private final Joystick flightStick = new Joystick(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
            drive = driveEnabled   ? instantiateRealDrive()  : null;
            intake = intakeEnabled ? instantiateRealIntake() : null;
            wrist = wristEnabled   ? instantiateRealWrist()  : null;
            break;

            case SIM:
            // Sim robot, instantiate physics sim IO implementations
            drive = driveEnabled   ? instantiateSimDrive()  : null;
            intake = intakeEnabled ? instantiateSimIntake() : null;
            wrist = wristEnabled   ? instantiateSimWrist()  : null;
            break;

            default:
            // Replayed robot, disable IO implementations
            drive = driveEnabled   ? instantiateDriveReplayed()  : null;
            intake = intakeEnabled ? instantiateIntakeReplayed() : null;
            wrist = wristEnabled   ? instantiateWristReplayed()  : null;
            break;
        }


    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());



    if(driveEnabled){
        driveConstructorStuff();
    }

    if(intakeEnabled){
        intakeConstructorStuff();
    }

    if(wristEnabled){
        wristConstructorStuff();
    }

    


    AutoBuilder.buildAutoChooser();


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

    if(driveEnabled){
        configureDriveBindings();
    }

    if(intakeEnabled){
        configureIntakeBindings();
    }

    if(wristEnabled){
        configureWristBindings();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.get();

  }






  // ============= Instantiation =============

  private Drive instantiateRealDrive(){
    // Real robot, instantiate hardware IO implementations
    return
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
  }
  private Drive instantiateSimDrive() {
    return new Drive(
        new GyroIO() {},
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim());
    }
    private Drive instantiateDriveReplayed() {
        return new Drive(
            new GyroIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {},
            new ModuleIO() {});
    }

    private Intake instantiateRealIntake(){
        return new Intake(100, 101, 102);
    }
    private Intake instantiateSimIntake(){
        return null;
    }
    private Intake instantiateIntakeReplayed(){
        return null;
    }

    private Wrist instantiateRealWrist(){
        return null;
    }
    private Wrist instantiateSimWrist(){
        return null;
    }
    private Wrist instantiateWristReplayed(){
        return null;
    }










    // ============= Constructor Stuff =============

    private void driveConstructorStuff() {
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


        autoChooser.addOption("Pathfinding with apriltag test",
            Commands.sequence(
                Commands.runOnce(() -> {
                    drive.setPose(new Pose2d(1, 1, new Rotation2d(Math.PI)));
                }, drive),
                Commands.defer(() -> {
                    return AutoFactoryGen2.PathfindRelativeToAprilTag(new Pose2d(0.7, 0, new Rotation2d()), drive);
                }, Set.of(drive))
            )
        );
    }

    private void intakeConstructorStuff() {
        // ...
    }

    private void wristConstructorStuff() {
        // ...
    }







    // ============= Bindings =============
    
    private void configureDriveBindings() {
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
    }
    private void configureIntakeBindings() {
        Trigger mainTrigger = new JoystickButton(flightStick, 1);
        mainTrigger.whileTrue(Commands.runOnce(() -> {intake.goToExtendedPosition();}, intake));
        mainTrigger.whileFalse(Commands.runOnce(() -> {intake.goToRetractedPosition();}, intake));
    }
    private void configureWristBindings() {
        // ...
    }
}
