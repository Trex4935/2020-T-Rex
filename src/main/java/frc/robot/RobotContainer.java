// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import Trex code
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Extensions.*;

import edu.wpi.first.wpilibj.DriverStation;
// import needed WPI methods
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Elevator elevator;

  // Commands
  private final DriveWithControllerCommand driveWithController;
  private final ShootCommand shoot;
  private final ReverseMagazineCommand reverseMagazine;
  private final ReverseMagazineCommand reverseMagazine2;
  private final SingulateBallCommand singulateBall;
  private final DriveStraightWithController driveStraightWithController;
  private final ElevatorUpCommand elevatorup;
  private final ElevatorDownCommand elevatordown;
  private final AutoTripleMagazineCommand autonomousTripleShoot;
  private final ShootPIDCommand shootPID;
  private final EmptyMagToShooterCommand emptyMag;
  public static int station;
  private final microAdjustCommand microAdjust;
  private final AutoDriveStraitCommand autoDriveStrait;
  private final AutoRotateDegrees autoRotateDegrees;

  public RobotContainer() {

    // Drivetrain
    driveTrain = new DriveTrain();
    driveWithController = new DriveWithControllerCommand(driveTrain);
    driveWithController.addRequirements(driveTrain);
    driveStraightWithController = new DriveStraightWithController(driveTrain);
    driveTrain.setDefaultCommand(driveWithController);
    microAdjust = new microAdjustCommand();
    autoDriveStrait = new AutoDriveStraitCommand(driveTrain, 100000, -0.5);
    autoRotateDegrees = new AutoRotateDegrees(driveTrain, 90);

    // Controller
    controller = new XboxController(Constants.xboxControllerPort);

    // Shooter
    shooter = new Shooter();
    shoot = new ShootCommand(shooter);
    shootPID = new ShootPIDCommand(shooter);

    // Magazine
    magazine = new Magazine();
    reverseMagazine = new ReverseMagazineCommand(magazine);
    reverseMagazine2 = new ReverseMagazineCommand(magazine);
    singulateBall = new SingulateBallCommand(magazine);

    emptyMag = new EmptyMagToShooterCommand(magazine);

    // Autonomous
    autonomousTripleShoot = new AutoTripleMagazineCommand(driveTrain, shooter, magazine);

    // Elevator
    elevator = new Elevator();
    elevatorup = new ElevatorUpCommand(elevator);
    elevatordown = new ElevatorDownCommand(elevator);

    // Station
    station = DriverStation.getInstance().getLocation();

    // Configure the button bindings
    configureButtonBindings();

  }

  // Setup controller bindings
  private void configureButtonBindings() {

    // Run intake ... stops when the shoot sensor is triggered
    new JoystickButton(controller, XboxController.Button.kB.value)
        .toggleWhenPressed(singulateBall.withInterrupt(Magazine::getShooterSensor).andThen(reverseMagazine2.withTimeout(0.1)).andThen(shoot));

    // Runs pulley + intake to reverse a ball thru the magazine
    new JoystickButton(controller, XboxController.Button.kX.value).whenHeld(reverseMagazine);

    new JoystickButton(controller, XboxController.Button.kA.value).toggleWhenPressed(autoRotateDegrees);

    // Makes sure the robot only goes straight by using right bumper
    new JoystickButton(controller, XboxController.Button.kBumperRight.value).whenHeld(driveStraightWithController);

    // Moves elevator up with D-Pad Up
    new POVButton(controller, 0).whileHeld(elevatorup);

    // Move elevator down with D-Pad down
    new POVButton(controller, 180).whileHeld(elevatordown);

    // Empties magazine using left trigger
    new LeftTriggerBool().whileActiveContinuous(emptyMag.alongWith(shootPID));
    // new
    // LeftTriggerBool().whenActive(reverseMagazine.withTimeout(0.1).andThen(emptyMag).alongWith(shootPID));

    // Runs magazine and sets shooter motor with PID -- DONT DELETE... possible
    // usage in the future
    /// new RightTriggerBool().whileActiveContinuous(highBelt.alongWith(shootPID2));

    // Available for micro-adjustments for shooting, intake, etc. Changes speed of
    // robot to a slower value.
    new RightTriggerBool().whileActiveContinuous(microAdjust);

    // whileActiveContinuous(emptyMag.alongWith(shootPID));

    /// CONTROLLER MAP
    //
    // A -
    // B - Turn on Intake
    // X - Reverse Magazine
    // Y -
    //
    // LT - Hold to spin up Shooter and Empty Magazine when shooter at speed
    // RT - Slows robot for micro-adjustments
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

    // new RamseteCommand(TrajectoryRepo.trajectory, driveTrain::getPose,
    // new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    // Constants.kDriveKinematics,
    // driveTrain::move, driveTrain);

    // Encoder Position, Encoder Speed, Gyro-Data based Trajectory Command
    // (TBD)

    // Ramsete Autonomous command;
    // Command autonomousCommand = ramseteCommand;
    // Command autonomousCommand = bouncePath;

    // Command autonomousCommand = autoShootSpeed;

    // Galactic Search Autonomous
    // Command autonomousCommand = autoAndMagazine;
    // Shooter SRuns Autonomous
    // Command autonomousCommand = autonomousSRunsAndShoot;

    // if (station==1) {
    // Command autonomousCommand = autonomousTripleShoot;
    // return autonomousCommand;
    // }
    // else if (station ==2) {
    // Command autonomousCommand = autoForward;
    // return autonomousCommand;
    // }
    // else {
    // Command autonomousCommand = autonomousTripleShoot;
    // return autonomousCommand;

    // }

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