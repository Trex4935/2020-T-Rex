// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import trex code
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Extensions.*;

// import needed WPI methods
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  public final DriveTrain driveTrain;
  private final DriveWithControllerCommand driveWithController;
  public static XboxController controller;
  private final BouncePathCommand bouncePath;
  private final Shooter shooter;
  private final ShootCommand shoot;
  public final Magazine magazine;
  private final ReverseMagazineCommand reverseMagazine;
  private final SingulateBallCommand singulateBall;
  private final DriveStraightWithController driveStraightWithController;
  private final HighBeltCommand highbelt;

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
    highbelt = new HighBeltCommand(magazine);

    
    // Autonomous
    bouncePath = new BouncePathCommand(driveTrain);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Runs pully + intake to move a ball thru the magazine
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(runBothMotors);

    // Runs pully + intake to reverse a ball thru the magazine
    new JoystickButton(controller, XboxController.Button.kX.value).whenHeld(reverseMagazine);

    // Run entire intake and magazine manually while button is held
    // new JoystickButton(controller, XboxController.Button.kY.value).whenHeld(runBothMotors);

    // Run intake only, on / off with B button press
    new JoystickButton(controller, XboxController.Button.kB.value).toggleWhenPressed(singulateBall.withInterrupt(Magazine::getShooterSensor));

    // Run magazine only, active only when A button held for manual singulation
    new JoystickButton(controller, XboxController.Button.kA.value).toggleWhenPressed(shoot);

    // Runs shooter motor when the right trigger is pulled
    // new RightTriggerBool().whileActiveContinuous(shootPID);

    // Uses limelight to aim at target when left trigger is pulled
    new LeftTriggerBool().whileActiveContinuous(highbelt);

    // Makes sure the robot only goes straight by using right bumper
    new JoystickButton(controller, XboxController.Button.kBumperRight.value).whenHeld(driveStraightWithController);

    // Emptys magazine using left bumper
    // new JoystickButton(controller, XboxController.Button.kBumperLeft.value).whenHeld(emptyMag.alongWith(shootPID));

    // Run the magazine + intake for a set time period
    // At the moment taking this off a button ... we need to figure out how to put this back!
    // new JoystickButton(controller, XboxController.Button.kY.value).whenPressed(runBothMotors.withTimeout(Constants.intakeTimeOut));

    // Runs shootPID when left trigger is pulled
   // new LeftTriggerBool().whileActiveContinuous(shootPID);

    // Run the magazine + intake when the intake sensor sees a ball
    // new IntakeTrigger().whenActive(runBothMotors.withTimeout(Constants.intakeTimeOut));
    
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
    //RamseteCommand ramseteCommand = new RamseteCommand(driveTrain.trajectory, driveTrain::getPosition, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics, driveTrain::move, driveTrain);
    
    new RamseteCommand(driveTrain.trajectory, driveTrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics,
        driveTrain::move, driveTrain);

    // Encoder Position, Encoder Speed, Gyro-Data based Trajectory Command 
    //(TBD)

    //Ramsete Autonomous command;
    //Command autonomousCommand = ramseteCommand; 
    Command autonomousCommand = bouncePath;

    //Command autonomousCommand = autoShootspeed;

    // Galatic Search Autonomous
    //Command autonomousCommand = autoAndMagazine;
    // Shooter SRuns Autonomous
    //Command autonomousCommand = autonomousSRunsAndShoot;

    return autonomousCommand;
  }
}