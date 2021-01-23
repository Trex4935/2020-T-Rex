// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.namespace.QName;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoForward;
import frc.robot.commands.DriveWithController;
<<<<<<< HEAD
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
=======
import frc.robot.commands.SpitBall;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Magazine;
>>>>>>> 9644f32dc2eb05f1b2610a792bec311d02a512a6
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
  private final DriveWithController driveWithController;
  public static XboxController controller;
  private final AutoForward autoFoward;
<<<<<<< HEAD
  private final Shooter shooter;
  private final Shoot shoot;
=======
  private final Magazine magazine;
  private final SpitBall spitBall;
>>>>>>> 9644f32dc2eb05f1b2610a792bec311d02a512a6

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Declare the drivetrain
    driveTrain = new DriveTrain();
    driveWithController = new DriveWithController(driveTrain);
    driveWithController.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithController);

    // Declare the controller
    controller = new XboxController(Constants.xboxControllerPort);

    // Declare auto method
    autoFoward = new AutoForward(driveTrain);

<<<<<<< HEAD
    // Set up shooter
    shooter = new Shooter();

    // Set up shoot
    shoot = new Shoot(shooter);
=======
    // Declare magazine
    magazine = new Magazine();
    spitBall = new SpitBall(magazine);
>>>>>>> 9644f32dc2eb05f1b2610a792bec311d02a512a6

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
    new JoystickButton(controller, XboxController.Button.kA.value).whenActive(new AutoForward(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoFoward;
  }
}
