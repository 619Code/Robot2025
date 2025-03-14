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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.Constants.WristConstants.WristAngleRad;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.OuttakeCoralCommand;
import frc.robot.commands.ElevatorCommands.ElevatorFineTuningCommand;
import frc.robot.commands.ElevatorCommands.ElevatorGoToPositionPositionCommand;
import frc.robot.commands.ElevatorCommands.ElevatorHoldCurrentPositionCommand;
import frc.robot.commands.WristCommands.WristGoToPositionCommand;
import frc.robot.commands.WristCommands.WristHoldCurrentPositionCommand;
import frc.robot.subsystems.IProfiledReset;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Manipulator.Manipulator;
import frc.robot.subsystems.WristStuff.Wrist;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Gyro.GyroIO;
import frc.robot.subsystems.drive.Gyro.GyroIONavX;
import frc.robot.subsystems.drive.Module.ModuleIOSim;
import frc.robot.subsystems.drive.Module.ModuleIOSpark;
import frc.robot.util.FunnelIntakeCommands;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {


    // Subsystems
    private final Drive drive;
//    private final Intake intake;
    private final Wrist wrist;
    private final Manipulator manipulator;
//    private final Passthrough passthrough;
    private final Elevator elevator;
 //   private final ServoSubsystem servo;
 //   private final ledSubsystem leds;
 //   private final Climb climb;

    private IProfiledReset[] subsystems;


    //private final boolean driveEnabled =            true;
    //private final boolean wristEnabled =            true;
    //private final boolean manipulatorEnabled =      true;
    // private final boolean intakeEnabled =           false;
    // private final boolean passthroughEnabled =      false;
   // private final boolean elevatorEnabled =         true;
    // private final boolean servoEnabled =            false;
    // private final boolean ledEnabled =              false;
   // private final boolean climbEnabled =            false;

    // Controller
    private final Joystick flightStick = new Joystick(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer() {

        if(Robot.isReal()){
            drive = instantiateRealDrive();
        }
        else{
            drive = instantiateSimOrReplayedDrive();
        }

        // intake = intakeEnabled              ? new Intake()  : null;
        wrist = new Wrist();
        manipulator = new Manipulator();
        // passthrough = passthroughEnabled    ? new Passthrough() : null;
        elevator = new Elevator();
        // servo = servoEnabled                ? new ServoSubsystem(0, 1) : null;
        // leds = ledEnabled                   ? new ledSubsystem() : null;
        // climb = climbEnabled                ? new Climb() : null;


        //  Should get called right after subsystems are instantiated


        constructorThings();

        //  Should the constructor and auto chooser/builder stuff be called in this order?

        driveConstructorStuff();
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());



        competitionButtonBindings();

    }


    private void constructorThings(){
        // if(intakeEnabled){
        //     intakeConstructorStuff();
        // }

        wristConstructorStuff();

        manipulatorConstructorStuff();

        elevatorConstructorStuff();


        // if(ledEnabled){
        //     ledConstructorStuff();
        // }

        // if(climbEnabled){
        //     climbConstructorStuff();
        // }

        registerNamedCommands();

    }






    private void competitionButtonBindings(){

        configureDriveBindings();

        //  ELEVATOR & WRIST

        Trigger dPadDown = operatorController.povDown();
        dPadDown.onTrue(robotGoToHeightCommandCreator(ElevatorHeight.L1));

        Trigger dPadRight = operatorController.povRight();
        dPadRight.onTrue(robotGoToHeightCommandCreator(ElevatorHeight.L2));

        Trigger dPadLeft = operatorController.povLeft();
        dPadLeft.onTrue(robotGoToHeightCommandCreator(ElevatorHeight.L3));

        Trigger dPadUp = operatorController.povUp();
        dPadUp.onTrue(robotGoToHeightCommandCreator(ElevatorHeight.L4));


        //  OUTTAKING
        // Trigger leftBumper = operatorController.leftBumper();
        // leftBumper.whileTrue(new IntakeCoralCommand(manipulator, passthrough));

        Trigger rightBumper = operatorController.rightBumper();
        rightBumper.whileTrue(new OuttakeCoralCommand(manipulator));


        //  FUNNEL INTAKING

        Trigger bButton = operatorController.b();
        bButton.onTrue(FunnelIntakeCommands.FunnelIntakeCommandCreator(elevator, wrist, manipulator));


        //  RETURN TO HOME

        Trigger xButton = operatorController.x();
        xButton.onTrue(robotGoToHeightCommandCreator(ElevatorHeight.HOME));

        //  ELEVATOR OVERRIDE

        Trigger leftStickDown = operatorController.leftStick();
        leftStickDown.whileTrue(new ElevatorFineTuningCommand(elevator, () -> -operatorController.getRawAxis(1)));

    }


    private void registerNamedCommands(){

        NamedCommands.registerCommand(
            "RobotToL1",
            robotGoToHeightCommandCreator(ElevatorHeight.L1)
        );

        NamedCommands.registerCommand(
            "RobotToL2",
            robotGoToHeightCommandCreator(ElevatorHeight.L2)
        );

        NamedCommands.registerCommand(
            "RobotToL3",
            robotGoToHeightCommandCreator(ElevatorHeight.L3)
        );

        NamedCommands.registerCommand(
            "RobotToL4",
            robotGoToHeightCommandCreator(ElevatorHeight.L4)
        );

        NamedCommands.registerCommand(
            "RobotOuttakeIndefinitely",
            new OuttakeCoralCommand(manipulator)
        );

        NamedCommands.registerCommand(
            "RobotToFunnelIntakeSequence",
            FunnelIntakeCommands.FunnelIntakeCommandCreator(elevator, wrist, manipulator)
        );

        NamedCommands.registerCommand(
            "WristToL2L3",
            new WristGoToPositionCommand(wrist, WristAngleRad.L2L3)
        );

        NamedCommands.registerCommand(
            "RobotToHome",
            robotGoToHeightCommandCreator(ElevatorHeight.HOME)
        );
    }






    private Command robotGoToHeightCommandCreator(ElevatorHeight height){


        Command command;

        //INEFFICIENT FOR GETTING FROM CORAL STATION
            command = Commands.sequence(
                new WristGoToPositionCommand(wrist, WristAngleRad.L2L3),
                new ElevatorGoToPositionPositionCommand(elevator, height),
                new WristGoToPositionCommand(wrist, getEndWristAngleForGivenElevatorHeight(height))
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);

       
        return command;

    }

    private WristAngleRad getEndWristAngleForGivenElevatorHeight(ElevatorHeight height){
        switch(height){
            case HOME:
                return WristAngleRad.FREEHANG;
            case FUNNEL:
                return WristAngleRad.FUNNEL_ANGLE;
            case L1:
                return WristAngleRad.L1;
            case L2:
                return WristAngleRad.L2L3;
            case L3:
                return WristAngleRad.L2L3;
            case L4:
                return WristAngleRad.L4;
            default:
                System.out.println("THIS SHOULD HAVE NEVER HAPPENED");
                return WristAngleRad.L2L3;
        }
    }


    // private void testingButtonBindings() {

    //     configureDriveBindings();


    //     // if(intakeEnabled){
    //     //     configureIntakeBindings();
    //     // }

    //     configureWristBindings();

    //     configureManipulatorBindings();

    //     configureElevatorBindings();

    //     // if(servoEnabled){
    //     //     configureServoBindings();
    //     // }

    //     // if(ledEnabled){
    //     //     configureLedBindings();
    //     // }

    //     // if(climbEnabled){
    //     //     configureClimbBindings();
    //     // }




    //     // Trigger bPressedTrigger = operatorController.b();
    //     // bPressedTrigger.onTrue(
    //     //     Commands.sequence(
    //     //         new WristGoToPositionCommand(wrist, WristAngleRad.PASSTHROUGH),
    //     //         new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.FUNNEL),
    //     //         new WristGoToPositionCommand(wrist, WristAngleRad.FUNNEL_ANGLE)
    //     //     )
    //     // );
    // }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }


    //  Should get called in teleopInit, and maybe autonomousInit
    public void ResetProfiledSubsystemsOnEnable(){
        wrist.ProfileReset();
        elevator.ProfileReset();
        manipulator.ProfileReset();
    }


  // ============= Instantiation =============


  //  DRIVE INSTANTIATION

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
                Constants.DriveConstants.frontLeftDriveAbsoluteEncoderOffsetRots,
                Constants.DriveConstants.frontLeftEncoderPositiveDirection), // FRONT LEFT
            new ModuleIOSpark(
                1,
                Constants.DriveConstants.frontRightDriveMotorInverted,
                Constants.DriveConstants.frontRightTurnMotorInverted,
                Constants.DriveConstants.frontRightTurnEncoderInverted,
                Constants.DriveConstants.frontRightDriveAbsoluteEncoderPort,
                Constants.DriveConstants.frontRightDriveAbsoluteEncoderOffsetRots,
                Constants.DriveConstants.frontRightEncoderPositiveDirection), // FRONT RIGHT
            new ModuleIOSpark(
                2,
                Constants.DriveConstants.backLeftDriveMotorInverted,
                Constants.DriveConstants.backLeftTurnMotorInverted,
                Constants.DriveConstants.backLeftTurnEncoderInverted,
                Constants.DriveConstants.backLeftDriveAbsoluteEncoderPort,
                Constants.DriveConstants.backLeftDriveAbsoluteEncoderOffsetRots,
                Constants.DriveConstants.backLeftEncoderPositiveDirection), // BACK LEFT
            new ModuleIOSpark(
                3,
                Constants.DriveConstants.backRightDriveMotorInverted,
                Constants.DriveConstants.backRightTurnMotorInverted,
                Constants.DriveConstants.backRightTurnEncoderInverted,
                Constants.DriveConstants.backRightDriveAbsoluteEncoderPort,
                Constants.DriveConstants.backRightDriveAbsoluteEncoderOffsetRots,
                Constants.DriveConstants.backRightEncoderPositiveDirection)); // BACK RIGHT
    }
    private Drive instantiateSimOrReplayedDrive() {
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim()
        );
    }

    // ============= Constructor Stuff =============

    private void driveConstructorStuff() {

    }

    // private void intakeConstructorStuff() {
    //     // ...
    // }

    // private void climbConstructorStuff(){
    //     // ...
    // }

    private void wristConstructorStuff() {
        wrist.setDefaultCommand(new WristHoldCurrentPositionCommand(wrist));
    }

    private void manipulatorConstructorStuff(){
        // ...
    }

    private void elevatorConstructorStuff(){
        elevator.setDefaultCommand(new ElevatorHoldCurrentPositionCommand(elevator));
    }

    // private void ledConstructorStuff(){
    //     leds.setColor(0, 0, 255);
    // }




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
                    () -> {
                        boolean isFlipped =
                        DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                        if(isFlipped){
                            drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d(Math.PI)));
                        }else{
                            drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
                        }
                    },
                    drive)
                .ignoringDisable(true));
    }

    // private void configureIntakeBindings() {
    //     Trigger mainTrigger = new JoystickButton(flightStick, 1);
    //     mainTrigger.whileTrue(Commands.runOnce(() -> {intake.goToExtendedPosition();}, intake));
    //     mainTrigger.whileFalse(Commands.runOnce(() -> {intake.goToRetractedPosition();}, intake));
    // }


    public enum INTAKE_POSITION{
        INTAKE,
        HALF_STOW,
        STOW
    }

    public enum CLIMB_POSITION{
        OUT,
        IN
    }

    //  Idea with wrist/elevator is that the operator will:
    //   Hold [left bumper] to enable input for carriage
    //   Then hit
    // private void configureWristBindings() {

    //     Trigger aPressedTrigger = operatorController.a();
    //     aPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.FREEHANG));

    //     Trigger bPressedTrigger = operatorController.b();
    //     bPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L1));

    //     // Trigger xPressedTrigger = operatorController.x();
    //     // xPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L2L3));

    //     Trigger yPressedTrigger = operatorController.y();
    //     yPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L4));
    // }

    // private void configureManipulatorBindings(){
    //     Trigger intakeCoralTrigger = operatorController.leftBumper();
    //     intakeCoralTrigger.whileTrue(new ManipulatorIntakeCoralCommand(manipulator));


    //     Trigger outtakeCoralTrigger = operatorController.rightBumper();
    //     outtakeCoralTrigger.whileTrue(new OuttakeCoralCommand(manipulator));

    //     Trigger dislodgeDownwardTrigger = operatorController.rightStick();
    //    dislodgeDownwardTrigger.whileTrue(new DislodgeAlgaeCommand(manipulator, false));

    //    Trigger dislodgeUpwardTrigger = operatorController.leftStick();
    //    dislodgeUpwardTrigger.whileTrue(new DislodgeAlgaeCommand(manipulator, true));
    // }

    // private void configureElevatorBindings(){

    //     Trigger elevatorDown = operatorController.x();
    //     elevatorDown.whileTrue(
    //         new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.HOME)
    //     );

    //     Trigger dPadDown = operatorController.povDown();
    //     dPadDown.whileTrue(
    //         new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L1)
    //     );

    //     Trigger dPadRight = operatorController.povRight();
    //     dPadRight.whileTrue(
    //         new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L2)
    //     );

    //     Trigger dPadLeft = operatorController.povLeft();
    //     dPadLeft.whileTrue(
    //         new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L3)
    //     );

    //     Trigger dPadUp = operatorController.povUp();
    //     dPadUp.whileTrue(
    //         new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L4)
    //     );
    // }

    // private void configureServoBindings(){

    //     // Trigger servoTrigger = operatorController.rightTrigger();
    //     // servoTrigger.whileTrue(
    //     //     new ServoGoToAngleCommand(servo, 120));

    //     // Trigger servoTrigger2 = operatorController.leftTrigger();
    //     // servoTrigger2.whileTrue(
    //     //     new ServoGoToAngleCommand(servo, 0));
    // }

    // private void configureLedBindings(){
    //     LedAnimationCommand ledCommand = new LedAnimationCommand(leds);
    //     ledCommand.schedule();
    // }

    // private void configureClimbBindings(){
    //     // Trigger yPressedTrigger = operatorController.y();
    //     // yPressedTrigger.onTrue(new ClimbCommand(climb, CLIMB_POSITION.OUT));

    //     // Trigger xPressedTrigger = operatorController.x();
    //     // xPressedTrigger.onTrue(new ClimbCommand(climb, CLIMB_POSITION.IN));
    // }
}
