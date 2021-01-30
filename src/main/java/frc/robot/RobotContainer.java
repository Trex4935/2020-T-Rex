// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Extensions.RightTriggerBool;
import frc.robot.commands.AutoForwardCommand;
import frc.robot.commands.DriveWithControllerCommand;
import frc.robot.commands.IntakeBallCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

import frc.robot.commands.SpitBallCommand;
import frc.robot.subsystems.Magazine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
<<<<<<< HEAD
import frc.robot.commands.RunBothMotorsCommand;
import frc.robot.commands.ReverseMagazineCommand;
=======
import edu.wpi.first.wpilibj2.command.button.Trigger;
>>>>>>> afdb8e0d7c2bbaf717342a3c3e56f0fe5bb1556d

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain driveTrain;
  private final DriveWithControllerCommand driveWithController;
  public static XboxController controller;
  private final AutoForwardCommand autoForward;
  private final Shooter shooter;
  private final ShootCommand shoot;
  private final Magazine magazine;
  private final SpitBallCommand spitBall;
  private final IntakeBallCommand intakeBall;
  private final ReverseMagazineCommand reverseMagazine;


   */
  public RobotContainer() {

    // Declare the drivetrain
    driveTrain = new DriveTrain();
    driveWithController = new DriveWithControllerCommand(driveTrain);
    driveWithController.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithController);

    // Declare the controller
    controller = new XboxController(Constants.xboxControllerPort);

    // Declare auto method
    autoForward = new AutoForwardCommand(driveTrain);

    // Set up shooter
    shooter = new Shooter();

    // Set up shoot
    shoot = new ShootCommand(shooter);

    // Declare magazine
    magazine = new Magazine();
    spitBall = new SpitBallCommand(magazine);
    intakeBall = new IntakeBallCommand(magazine);
<<<<<<< HEAD
    runBothMotors = new RunBothMotorsCommand(magazine);
    reverseMagazine = new ReverseMagazineCommand(magazine);
=======
>>>>>>> afdb8e0d7c2bbaf717342a3c3e56f0fe5bb1556d

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
    // Turn the shooter on
    // Hold the Button down and it stays on .... one button on / one button off
<<<<<<< HEAD
    
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(intakeBall);
    // new JoystickButton(controller, XboxController.Button.kX.value).whenHeld(spitBall);

    new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(runBothMotors);
    new JoystickButton(controller, XboxController.Button.kX.value).whenHeld(reverseMagazine);
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(new ParallelCommandGroup(intakeBall,spitBall));
   
    //new Trigger(()->controller.getRawAxis(3)>=0.25).whileActiveContinuous(shoot);
=======
    new JoystickButton(controller, XboxController.Button.kA.value).whenActive(intakeBall);

    // new Trigger(()->controller.getRawAxis(3)>=0.25).whileActiveContinuous(shoot);
>>>>>>> afdb8e0d7c2bbaf717342a3c3e56f0fe5bb1556d
    new RightTriggerBool().whileActiveContinuous(shoot);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoForward;
  }
}