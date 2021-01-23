// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  // Declare all of our variables
  
  // Motors
  WPI_TalonFX leftFront;
  WPI_TalonFX rightFront;
  WPI_TalonFX leftRear;
  WPI_TalonFX rightRear;

  // Controllers
  SpeedControllerGroup rightSide;
  SpeedControllerGroup leftSide;

  // Drives
  DifferentialDrive drive;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Setup each of the motors for use later
    // Going to set any whole game settings here as well (like moter inversion)
    leftFront = new WPI_TalonFX(Constants.leftFrontCanID);
    leftFront.setInverted(false);

    rightFront = new WPI_TalonFX(Constants.rightFrontCanID);
    rightFront.setInverted(false);

    leftRear = new WPI_TalonFX(Constants.leftRearCanID);
    leftRear.setInverted(false);

    rightRear = new WPI_TalonFX(Constants.rightRearCanID);
    rightRear.setInverted(false);

    // create the speed controller groups for use in the differential drive
    // each one should be a pairing of the motors on a given side of the robot
    rightSide = new SpeedControllerGroup(rightFront, rightRear);
    leftSide = new SpeedControllerGroup(leftFront, leftRear);

    // create the drive object that will control the differential drive
    // It needs both a set of left and right motors
    // file:///C:/Users/Public/wpilib/2021/documentation/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.html
    drive = new DifferentialDrive(rightSide, leftSide);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Method to control the drive with the controller
  // controller = Input controller
  // speedLimiter = value to limit the speed of the motors
  public void driveWithController(XboxController controller, double speedLimiter)
  {
  if (Constants.arcadeDrive) {
    drive.arcadeDrive(controller.getRawAxis(Constants.leftTankAxis)*speedLimiter, controller.getRawAxis(Constants.rightArcadeAxis)*speedLimiter);
  } else {
    drive.tankDrive(controller.getRawAxis(Constants.leftTankAxis)*speedLimiter, controller.getRawAxis(Constants.rightTankAxis)*speedLimiter);
  }
    
  }

  // Move us forward during auto
  public void autoForward(double seconds)
  {
    drive.tankDrive(Constants.autoLeftSpeed, Constants.autoRightSpeed);
  }

  // Method to just stop the drive
  public void stop(){
    drive.stopMotor();
  }

}
