// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import trex code
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Extensions.RightTriggerBool;

// import needed WPI methods
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final DriveTrain driveTrain;
  private final DriveWithControllerCommand driveWithController;
  public static XboxController controller;
  private final AutoForwardCommand autoForward;
  private final Shooter shooter;
  private final ShootCommand shoot;
  private final Magazine magazine;
  private final ReverseMagazineCommand reverseMagazine;
  private final RunBothMotorsCommand runBothMotors;


  public RobotContainer() {

    // Drivetrain
    driveTrain = new DriveTrain();
    driveWithController = new DriveWithControllerCommand(driveTrain);
    driveWithController.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(driveWithController);

    // Controller
    controller = new XboxController(Constants.xboxControllerPort);

    // Autonomous
    autoForward = new AutoForwardCommand(driveTrain);

    // Shooter
    shooter = new Shooter();
    shoot = new ShootCommand(shooter);

    // Magazine
    magazine = new Magazine();
    runBothMotors = new RunBothMotorsCommand(magazine);
    reverseMagazine = new ReverseMagazineCommand(magazine);

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
    new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(runBothMotors);

    // Runs pully + intake to reverse a ball thru the magazine
    new JoystickButton(controller, XboxController.Button.kX.value).whenHeld(reverseMagazine);

    // Runs shooter motor when the right trigger is pulled
    new RightTriggerBool().whileActiveContinuous(shoot);


    // Not using
    // new Trigger(()->controller.getRawAxis(3)>=0.25).whileActiveContinuous(shoot);
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(intakeBall);
    // new JoystickButton(controller, XboxController.Button.kX.value).whenHeld(spitBall);
    // new JoystickButton(controller, XboxController.Button.kA.value).whenHeld(new ParallelCommandGroup(intakeBall,spitBall));

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