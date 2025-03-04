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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.commands.DislodgeAlgaeCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCoralCommand;
import frc.robot.commands.OuttakeCoralCommand;
import frc.robot.commands.ServoGoToAngleCommand;
import frc.robot.commands.ElevatorCommands.ElevatorGoToPositionPositionCommand;
import frc.robot.commands.ElevatorCommands.ElevatorHoldCurrentPositionCommand;
import frc.robot.commands.WristCommands.WristGoToPositionCommand;
import frc.robot.commands.WristCommands.WristHoldCurrentPositionCommand;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorIOReal;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.FunnelCollapser.ServoSubsystem;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Outtake.Manipulator;
import frc.robot.subsystems.Outtake.ManipulatorIOReal;
import frc.robot.subsystems.Passthrough.Passthrough;
import frc.robot.subsystems.WristStuff.Wrist;
import frc.robot.subsystems.WristStuff.WristIOReal;
import frc.robot.subsystems.WristStuff.WristIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Gyro.GyroIO;
import frc.robot.subsystems.drive.Gyro.GyroIONavX;
import frc.robot.subsystems.drive.Module.ModuleIOSim;
import frc.robot.subsystems.drive.Module.ModuleIOSpark;


import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


public class RobotContainer {


    // Subsystems
    private final Drive drive;
    private final Intake intake;
    private final Wrist wrist;
    private final Manipulator manipulator;
    private final Passthrough passthrough;
    private final Elevator elevator;
    private final ServoSubsystem servo;


    private final boolean driveEnabled = true;
    private final boolean wristEnabled = true;
    private final boolean manipulatorEnabled = false;
    private final boolean intakeEnabled = false;
    private final boolean passthroughEnabled = false;
    private final boolean elevatorEnabled = true;
    private final boolean servoEnabled = false;


    // Controller
    private final Joystick flightStick = new Joystick(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;


    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                drive = driveEnabled     ? instantiateRealDrive()   : null;
                intake = intakeEnabled   ? instantiateRealIntake()  : null;
                wrist = wristEnabled     ? instantiateRealWrist()   : null;
                manipulator = manipulatorEnabled ? instantiateRealManipulator() : null;
                passthrough = passthroughEnabled ? instantiateRealPassthrough() : null;
                elevator = elevatorEnabled ? instantiateRealElevator() : null;
                servo = servoEnabled ? instantiateRealServo() : null;

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = driveEnabled   ? instantiateSimDrive()  : null;
                intake = intakeEnabled ? instantiateSimIntake() : null;
                wrist = wristEnabled   ? instantiateSimWrist()  : null;
                manipulator = manipulatorEnabled ? instantiateSimManipulator() : null;
                passthrough = passthroughEnabled ? instantiateSimPassthrough() : null;
                elevator = elevatorEnabled ? instantiateSimElevator() : null;
                servo = servoEnabled ? instantiateSimServo() : null;

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = driveEnabled   ? instantiateDriveReplayed()  : null;
                intake = intakeEnabled ? instantiateIntakeReplayed() : null;
                wrist = wristEnabled   ? instantiateWristReplayed()  : null;
                manipulator = manipulatorEnabled ? instantiateManipulatorReplayed() : null;
                passthrough = passthroughEnabled ? instantiateReplayedPassthrough() : null;
                elevator = elevatorEnabled ? instantiateElevatorReplayed() : null;
                servo = servoEnabled ? instantiateReplayedServo() : null;

                break;
        }

        // if(driveEnabled){
        //     driveConstructorStuff();
        //     autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        //     AutoBuilder.buildAutoChooser();
        // }else{
        //     autoChooser = null;
        // }

        constructorThings();

        if(driveEnabled){
            driveConstructorStuff();
            autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
            AutoBuilder.buildAutoChooser();
        }else{
            autoChooser = null;
        }

        configureButtonBindings();
    }


    private void constructorThings(){
        if(intakeEnabled){
            intakeConstructorStuff();
        }

        if(wristEnabled){
            wristConstructorStuff();
        }

        if(manipulatorEnabled){
            manipulatorConstructorStuff();
        }

        if(elevatorEnabled){
            elevatorConstructorStuff();
        }
    }




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

        if(manipulatorEnabled){
            configureManipulatorBindings();
        }

        if(elevatorEnabled){
            configureElevatorBindings();
        }

        if(servoEnabled){
            configureServoBindings();
        }
    }

    public Command getAutonomousCommand() {
        if(driveEnabled){
            return autoChooser.get();
        }else{
            return null;
        }
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
            new ModuleIOSim()
        );
    }
    private Drive instantiateDriveReplayed() {
        return new Drive(
            new GyroIO() {},
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim()
        );
    }

    //  INTAKE INSTANTIATION

    private Intake instantiateRealIntake(){
        return new Intake(100, 101, 102);
    }
    private Intake instantiateSimIntake(){
        return new Intake(103, 104, 105);
    }
    private Intake instantiateIntakeReplayed(){
        return null;
    }

    //  WRIST INSTANTIATION

    private Wrist instantiateRealWrist(){
        return new Wrist(new WristIOReal());
    }
    private Wrist instantiateSimWrist(){
        return new Wrist(new WristIOSim());
    }
    private Wrist instantiateWristReplayed(){
        return new Wrist(new WristIOSim());
    }

    //  MANIPULATOR INSTANTIATION

    private Manipulator instantiateRealManipulator(){
        return new Manipulator(new ManipulatorIOReal());
    }
    private Manipulator instantiateSimManipulator(){
        throw new UnsupportedOperationException("NOT IMPLEMENTED");
    }
    private Manipulator instantiateManipulatorReplayed(){
        throw new UnsupportedOperationException("NOT IMPLEMENTED");
    }

    //PASSTHROUGH INSTANTIATION
    private Passthrough instantiateRealPassthrough(){
    //TODO: Change these
        return new Passthrough(-1, -1);
    }
    private Passthrough instantiateSimPassthrough(){
        return new Passthrough(-1, -1);
    }
    private Passthrough instantiateReplayedPassthrough(){
            throw new UnsupportedOperationException("NOT IMPLEMENTED");
        }

    //  ELEVATOR INSTANTIATION
    private Elevator instantiateRealElevator(){
        return new Elevator(new ElevatorIOReal());
    }
    private Elevator instantiateSimElevator(){
        return new Elevator(new ElevatorIOSim());
    }
    private Elevator instantiateElevatorReplayed(){
        return null;
    }

    //SERVO INSTANTIATION
    private ServoSubsystem instantiateRealServo() {
        return new ServoSubsystem(0);
    }
    private ServoSubsystem instantiateSimServo() {
        throw new UnsupportedOperationException("NOT IMPLEMENTED");
    }
    private ServoSubsystem instantiateReplayedServo() {
        throw new UnsupportedOperationException("NOT IMPLEMENTED");
    }


    // ============= Constructor Stuff =============

    private void driveConstructorStuff() {

    }

    private void intakeConstructorStuff() {
        // ...
    }

    private void wristConstructorStuff() {
        wrist.setDefaultCommand(new WristHoldCurrentPositionCommand(wrist));
        NamedCommands.registerCommand("SetWristAnglePassthrough",
            new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.PASSTHROUGH));
        NamedCommands.registerCommand("SetWristAngleL1",
            new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L1));
        NamedCommands.registerCommand("SetWristAngleL2L3",
            new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L2L3));
        NamedCommands.registerCommand("SetWristAngleL4",
            new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L4));
    }

    private void manipulatorConstructorStuff(){
        NamedCommands.registerCommand("IntakeCoral",
            new IntakeCoralCommand(manipulator, passthrough));
        NamedCommands.registerCommand("OuttakeCoral",
            new OuttakeCoralCommand(manipulator));
        NamedCommands.registerCommand("DislodgeAlageDownward",
            new DislodgeAlgaeCommand(manipulator, false));
        NamedCommands.registerCommand("DislodgeAlageUpward",
            new DislodgeAlgaeCommand(manipulator, true));
    }

    private void elevatorConstructorStuff(){
        elevator.setDefaultCommand(new ElevatorHoldCurrentPositionCommand(elevator));
        NamedCommands.registerCommand("ElevatorToPassthrough",
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.PASSTHROUGH));
        NamedCommands.registerCommand("ElevatorToL1",
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L1));
        NamedCommands.registerCommand("ElevatorToL2",
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L2));
        NamedCommands.registerCommand("ElevatorToL3",
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L3));
        NamedCommands.registerCommand("ElevatorToL4",
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L4));
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


    public enum INTAKE_POSITION{
        INTAKE,
        CLIMB,
        STORE
    }

    //  Idea with wrist/elevator is that the operator will:
    //   Hold [left bumper] to enable input for carriage
    //   Then hit
    private void configureWristBindings() {

        Trigger aPressedTrigger = operatorController.a();
        aPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.PASSTHROUGH));

        Trigger bPressedTrigger = operatorController.b();
        bPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L1));

        Trigger xPressedTrigger = operatorController.x();
        xPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L2L3));

        Trigger yPressedTrigger = operatorController.y();
        yPressedTrigger.onTrue(new WristGoToPositionCommand(wrist, Constants.WristConstants.WristAngleRad.L4));
    }

    private void configureManipulatorBindings(){
        Trigger intakeCoralTrigger = operatorController.leftBumper();
        if (elevator.getPositionMeters() == Constants.ElevatorConstants.ElevatorHeight.PASSTHROUGH.heightMeters
        && wrist.getPosition() == Constants.WristConstants.WristAngleRad.PASSTHROUGH.positionRad) {
        intakeCoralTrigger.whileTrue(new IntakeCoralCommand(manipulator, passthrough));
        }

        Trigger outtakeCoralTrigger = operatorController.rightBumper();
        outtakeCoralTrigger.whileTrue(new OuttakeCoralCommand(manipulator));

        Trigger dislodgeDownwardTrigger = operatorController.rightStick();
       dislodgeDownwardTrigger.whileTrue(new DislodgeAlgaeCommand(manipulator, false));

       Trigger dislodgeUpwardTrigger = operatorController.leftStick();
       dislodgeUpwardTrigger.whileTrue(new DislodgeAlgaeCommand(manipulator, true));
    }

    private void configureElevatorBindings(){

        Trigger elevatorToPassthroughHeight = operatorController.povDown();
        elevatorToPassthroughHeight.whileTrue(
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.PASSTHROUGH)
        );

        Trigger elevatorToL2Height = operatorController.povRight();
        elevatorToL2Height.whileTrue(
            new ElevatorGoToPositionPositionCommand(elevator, ElevatorHeight.L2)
        );

    }

    private void configureServoBindings(){

        Trigger servoTrigger = operatorController.rightTrigger();
        servoTrigger.whileTrue(
            new ServoGoToAngleCommand(servo, 120));

        Trigger servoTrigger2 = operatorController.leftTrigger();
        servoTrigger2.whileTrue(
            new ServoGoToAngleCommand(servo, 0));
    }
}
