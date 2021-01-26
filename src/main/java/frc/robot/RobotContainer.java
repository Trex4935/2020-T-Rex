// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
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
    // Turn the shooter on
    // Hold the Button down and it stays on .... one button on / one button off
<<<<<<< HEAD
    new JoystickButton(controller, XboxController.Button.kA.value).whenActive(intakeBall);
=======
    new JoystickButton(controller, XboxController.Axis.kRightTrigger.value).whenHeld(shoot);
>>>>>>> 606f024f55196175e8fb44a14ce8caf150bbf623
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