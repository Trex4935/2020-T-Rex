// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import Trex code
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Extensions.*;

// import needed WPI methods
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...

  // Subsystems
  public final DriveTrain driveTrain;
  public final Magazine magazine;
  private final Shooter shooter;
  public static XboxController controller;

  // Commands
  private final DriveWithControllerCommand driveWithController;
  private final BouncePathCommand bouncePath;
  private final ShootCommand shoot;
  private final ReverseMagazineCommand reverseMagazine;
  private final SingulateBallCommand singulateBall;
  private final DriveStraightWithController driveStraightWithController;
  private final HighBeltCommand highBelt;
  private final Elevator elevator;
  private final ElevatorUpCommand elevatorup;
  private final ElevatorDownCommand elevatordown;
  private final ElevSolenoidCommand elevatorsolenoid;
  private final AutoTripleMagazineCommand autonomousTripleShoot;


  public RobotContainer() {

    // Drivetrain
    driveTrain = new DriveTrain();
    driveWithController = new DriveWithControllerCommand(driveTrain);
    driveWithController.addRequirements(driveTrain);
    driveStraightWithController = new DriveStraightWithController(driveTrain);
    driveTrain.setDefaultCommand(driveWithController);

    // Controller
    controller = new XboxController(Constants.xboxControllerPort);

    // Shooter
    shooter = new Shooter();
    shoot = new ShootCommand(shooter);

    // Magazine
    magazine = new Magazine();
    reverseMagazine = new ReverseMagazineCommand(magazine);
    singulateBall = new SingulateBallCommand(magazine);
    highBelt = new HighBeltCommand(magazine);

    // Autonomous
    bouncePath = new BouncePathCommand(driveTrain);
    autonomousTripleShoot = new AutoTripleMagazineCommand(driveTrain, shooter, magazine);

    // Elevator
    elevator = new Elevator();
    elevatorup = new ElevatorUpCommand(elevator);
    elevatordown = new ElevatorDownCommand(elevator);
    elevatorsolenoid = new ElevSolenoidCommand(elevator);


    // Configure the button bindings
    configureButtonBindings();

  }

  // Setup controller bindings
  private void configureButtonBindings() {

    // Turn on the shooter when toggles
    new JoystickButton(controller, XboxController.Button.kA.value).toggleWhenPressed(shoot);

    // Run intake ... stops when the shoot sensor is triggered
    new JoystickButton(controller, XboxController.Button.kB.value)
        .toggleWhenPressed(singulateBall.withInterrupt(Magazine::getShooterSensor));

    // Runs pulley + intake to reverse a ball thru the magazine
    new JoystickButton(controller, XboxController.Button.kX.value).whenHeld(reverseMagazine);

    // Runs shooter motor when the right trigger is pulled
    // new RightTriggerBool().whileActiveContinuous(shootPID);

    // Uses limelight to aim at target when left trigger is pulled
    new LeftTriggerBool().whileActiveContinuous(highBelt);

    // Makes sure the robot only goes straight by using right bumper
    new JoystickButton(controller, XboxController.Button.kBumperRight.value).whenHeld(driveStraightWithController);
    
    // Using ElevatorUp() command mapping to the back button on the controller.
    new  POVButton(controller, 0).whileHeld(elevatorup);
 
    // Using ElevatorDown() command mapping to the back button on the controller.
    new  POVButton(controller, 180).whileHeld(elevatordown);

    // Open the solenoid on elevator
    new JoystickButton(controller, XboxController.Button.kBack.value).toggleWhenPressed(elevatorsolenoid);

    /// CONTROLLER MAP
    //
    // A - Turn on Shooter
    // B - Turn on Intake
    // X - Reverse Magazine
    // Y -
    //
    // LT - When Held Run High Belt
    // RT -
    //
    // LB -
    // RB - Hold to drive straight
    //
    // LStick - Control left side drive train
    // RStick - Control right side drive train
    //
    // Start -
    // Select -
    //
    // D-Pad
    // Up - Move elevator Up
    // Right -
    // Down - Move elevator down
    // Left -
    //
    /// END MAP


    // Empties magazine using left bumper
    // new JoystickButton(controller,
    // XboxController.Button.kBumperLeft.value).whenHeld(emptyMag.alongWith(shootPID));

    // Run the magazine + intake for a set time period
    // At the moment taking this off a button ... we need to figure out how to put
    // this back!
    // new JoystickButton(controller,
    // XboxController.Button.kY.value).whenPressed(runBothMotors.withTimeout(Constants.intakeTimeOut));

    // Runs shootPID when left trigger is pulled
    // new LeftTriggerBool().whileActiveContinuous(shootPID);

    // Run the magazine + intake when the intake sensor sees a ball
    // new
    // IntakeTrigger().whenActive(runBothMotors.withTimeout(Constants.intakeTimeOut));

    // Not using
    // new Trigger(()->controller.getRawAxis(3)>=0.25).whileActiveContinuous(shoot);
    // new JoystickButton(controller,
    // XboxController.Button.kA.value).whenHeld(intakeBall);
    // new JoystickButton(controller,
    // XboxController.Button.kX.value).whenHeld(spitBall);
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(new
    // ParallelCommandGroup(intakeBall,spitBall));

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // An ExampleCommand will run in autonomous
    DriveTrain.resetTime();

    // Simulated Trajectory Command
    // RamseteCommand ramseteCommand = new RamseteCommand(TrajectoryRepo.trajectory,
    // driveTrain::getPosition, new RamseteController(Constants.kRamseteB,
    // Constants.kRamseteZeta), Constants.kDriveKinematics, driveTrain::move,
    // driveTrain);

    //new RamseteCommand(TrajectoryRepo.trajectory, driveTrain::getPose,
   //     new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics,
   //     driveTrain::move, driveTrain);

    // Encoder Position, Encoder Speed, Gyro-Data based Trajectory Command
    // (TBD)

    // Ramsete Autonomous command;
    // Command autonomousCommand = ramseteCommand;
    //Command autonomousCommand = bouncePath;

    // Command autonomousCommand = autoShootSpeed;

    // Galactic Search Autonomous
    // Command autonomousCommand = autoAndMagazine;
    // Shooter SRuns Autonomous
    // Command autonomousCommand = autonomousSRunsAndShoot;
    Command autonomousCommand = autonomousTripleShoot;

    return autonomousCommand;
  }
}

/// Code from the controlls for possible later use

// Empties magazine using left bumper
// new JoystickButton(controller,
// XboxController.Button.kBumperLeft.value).whenHeld(emptyMag.alongWith(shootPID));

// Run the magazine + intake for a set time period
// At the moment taking this off a button ... we need to figure out how to put
// this back!
// new JoystickButton(controller,
// XboxController.Button.kY.value).whenPressed(runBothMotors.withTimeout(Constants.intakeTimeOut));

// Runs shootPID when left trigger is pulled
// new LeftTriggerBool().whileActiveContinuous(shootPID);

// Run the magazine + intake when the intake sensor sees a ball
// new
// IntakeTrigger().whenActive(runBothMotors.withTimeout(Constants.intakeTimeOut));

// Not using
// new Trigger(()->controller.getRawAxis(3)>=0.25).whileActiveContinuous(shoot);
// new JoystickButton(controller,
// XboxController.Button.kA.value).whenHeld(intakeBall);
// new JoystickButton(controller,
// XboxController.Button.kX.value).whenHeld(spitBall);
// new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(new
// ParallelCommandGroup(intakeBall,spitBall));