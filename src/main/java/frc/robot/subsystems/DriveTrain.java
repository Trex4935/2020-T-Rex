// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Extensions.Dashboard_Outputs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;

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

  // Gyro
  AHRS ahrs;

  // Drive Type
  Dashboard_Outputs dashOut;

  // Trajectory
  Trajectory trajectory;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Setup each of the motors for use later
    // Going to set any whole game settings here as well (like motor inversion)
    leftFront = new WPI_TalonFX(Constants.leftFrontCanID);
    leftFront.setInverted(true);

    rightFront = new WPI_TalonFX(Constants.rightFrontCanID);
    rightFront.setInverted(true);

    leftRear = new WPI_TalonFX(Constants.leftRearCanID);
    leftRear.setInverted(true);

    rightRear = new WPI_TalonFX(Constants.rightRearCanID);
    rightRear.setInverted(true);

    ahrs = new AHRS(SPI.Port.kMXP);

    // create the speed controller groups for use in the differential drive
    // each one should be a pairing of the motors on a given side of the robot
    rightSide = new SpeedControllerGroup(rightFront, rightRear);
    leftSide = new SpeedControllerGroup(leftFront, leftRear);

    // create the drive object that will control the differential drive
    // It needs both a set of left and right motors
    // file:///C:/Users/Public/wpilib/2021/documentation/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.html
    drive = new DifferentialDrive(rightSide, leftSide);

    dashOut = new Dashboard_Outputs();

    // Trajectory

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts,
                                  Constants.kvVoltSecondsPerMeter,
                                  Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);

    
    // Create config for trajectory
    TrajectoryConfig config =
    new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                        Constants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
//     trajectory = TrajectoryGenerator.generateTrajectory(
//       // Start at the origin facing the +X direction
//       new Pose2d(0, 0, new Rotation2d(0)),
//       // Pass through these two interior waypoints, making an 's' curve path
//       List.of(
//           new Translation2d(1, 1),
//           new Translation2d(2, -1)
//       ),
//       // End 3 meters straight ahead of where we started, facing forward
//       new Pose2d(3, 0, new Rotation2d(0)),
//       // Pass config
//       config
// );
        
// Doc on how to access the file via the Robo Rio
// https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html
String trajectoryJSON = "..\\.\\deploy\\paths\\Unnamed.wpilib.json";
trajectory = new Trajectory();
try {
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex) {
  DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
}

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Method to control the drive with the controller
  // controller = Input controller
  // speedLimiter = value to limit the speed of the motors
  // if else statement to swap between arcade and tank
  public void driveWithController(XboxController controller, double speedLimiter) {
    if (dashOut.getDriveType()) {
      drive.arcadeDrive(controller.getRawAxis(Constants.leftTankAxis) * speedLimiter,
          controller.getRawAxis(Constants.rightArcadeAxis) * speedLimiter);
    } else {
      drive.tankDrive(controller.getRawAxis(Constants.leftTankAxis) * speedLimiter,
          controller.getRawAxis(Constants.rightTankAxis) * speedLimiter);
      // Added SmartDashboard support to read out controller data - Edit by Smiths
      // SmartDashboard.putNumber("Drive Left Axis", controller.getRawAxis(Constants.leftTankAxis));
      // SmartDashboard.putNumber("Drive Right Axis", controller.getRawAxis(Constants.rightTankAxis));
      // SmartDashboard.putNumber("Gyro Angle", ahrs.getAngle());
    }

  }

  // Move us forward during auto
  public void autoForward(double seconds) {
    drive.tankDrive(Constants.autoLeftSpeed, Constants.autoRightSpeed);
  }

  // Method to just stop the drive
  public void stop() {
    drive.stopMotor();
  }

  //Print out way points
  public void getWP(double time) {
    System.out.println(time);
    System.out.println(trajectory.sample(time));
  }

}
