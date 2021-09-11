// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TrajectoryRepo;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Extensions.DriveEncoders;
import frc.robot.Extensions.Limelight;

public class DriveTrain extends SubsystemBase {
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
  public static AHRS ahrs;

  // Odometry
  DifferentialDriveOdometry odometry;

  // Internal time
  public static double time;

  // Encoders object
  public static DriveEncoders driveEncoders;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Setup each of the motors for use later
    // Going to set any whole game settings here as well (like motor inversion)
    leftFront = new WPI_TalonFX(Constants.leftFrontCanID);
    leftFront.setInverted(Constants.inversion);
    // leftFront.configOpenloopRamp(Constants.openLoopRamp);

    rightFront = new WPI_TalonFX(Constants.rightFrontCanID);
    rightFront.setInverted(Constants.inversion);
    // rightFront.configOpenloopRamp(Constants.openLoopRamp);

    leftRear = new WPI_TalonFX(Constants.leftRearCanID);
    leftRear.setInverted(Constants.inversion);
    // leftRear.configOpenloopRamp(Constants.openLoopRamp);

    rightRear = new WPI_TalonFX(Constants.rightRearCanID);
    rightRear.setInverted(Constants.inversion);
    // rightRear.configOpenloopRamp(Constants.openLoopRamp);

    // Create the encoders object
    driveEncoders = new DriveEncoders();

    // create the speed controller groups for use in the differential drive
    // each one should be a pairing of the motors on a given side of the robot
    rightSide = new SpeedControllerGroup(rightFront, rightRear);
    leftSide = new SpeedControllerGroup(leftFront, leftRear);

    // create the drive object that will control the differential drive
    // It needs both a set of left and right motors
    // file:///C:/Users/Public/wpilib/2021/documentation/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.html
    drive = new DifferentialDrive(rightSide, leftSide);

    // Init the gyro
    ahrs = new AHRS(SPI.Port.kMXP);

    // Odometry
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getGyroAngle()), Constants.startPosition);
    System.out.println(ahrs.isCalibrating());
    // Trajectory
    time = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(ahrs.getRotation2d(),
        ticksToPosition(leftFront.getSelectedSensorPosition(), Constants.wheelDiameter, Constants.driveTrainGearRatio),
        ticksToPosition(-rightFront.getSelectedSensorPosition(), Constants.wheelDiameter,
            Constants.driveTrainGearRatio));

    driveEncoders.SetDriveEncoders(leftFront.getSelectedSensorPosition(), leftRear.getSelectedSensorPosition(),
        rightFront.getSelectedSensorPosition(), rightRear.getSelectedSensorPosition());
  }

  // Method to control the drive with the controller
  // controller = Input controller
  // speedLimiter = value to limit the speed of the motors
  // if else statement to swap between arcade and tank
  public void driveWithController(XboxController controller, double speedLimiter) {
    drive.tankDrive(controller.getRawAxis(Constants.leftTankAxis) * speedLimiter * Constants.driveDirection,
        controller.getRawAxis(Constants.rightTankAxis) * speedLimiter * Constants.driveDirection);

  }

  // Duplicate the right controller output onto left and right tank drive
  public void driveStraitWithController(XboxController controller, double speedLimiter) {
    double rightSpeed = controller.getRawAxis(Constants.rightTankAxis) * speedLimiter * Constants.driveDirection;
    drive.tankDrive(rightSpeed, rightSpeed);
  }

  // Move us forward during auto
  public void autoForward(double seconds) {
    drive.tankDrive(Constants.autoLeftSpeed, Constants.autoRightSpeed);
  }

  public static void resetTime() {
    time = 0;
  }

  public void initOdometry() {
    resetEncoders();
    odometry.resetPosition(new Pose2d(0, 0, new Rotation2d()), Rotation2d.fromDegrees(-getGyroAngle()));
  }

  public void resetOdometry(Pose2d startPosition) {
    resetEncoders();
    odometry.resetPosition(startPosition, Rotation2d.fromDegrees(-getGyroAngle()));
  }

  public void resetGyro() {
    ahrs.reset();
  }

  // Method to just stop the drive
  public void stopDriveTrain() {
    drive.stopMotor();
  }

  // Print out way points
  public void getWP(double time) {
    System.out.println(time);
    System.out.println(TrajectoryRepo.trajectory.sample(time));
  }

  public Pose2d getPosition() {
    time += 0.02;
    System.out.println(TrajectoryRepo.trajectory.sample(time).poseMeters);
    System.out.println(TrajectoryRepo.trajectory.sample(time).velocityMetersPerSecond);
    // Pose2d temp = getPose();
    return TrajectoryRepo.trajectory.sample(time).poseMeters;
  }

  // Takes in speed setpoints,convert them to volts and drive robot
  public void move(double LeftSpeed, double RightSpeed) {

    rightSide.setVoltage(-(LeftSpeed / Constants.kvVoltSecondsPerMeter) * Constants.autoCalibrate * (1)); // Or 12 or
                                                                                                          // kvVoltSecondsPerMeter
                                                                                                          // *WheelRatio
    leftSide.setVoltage((RightSpeed / Constants.kvVoltSecondsPerMeter) * Constants.autoCalibrate * (1)); // Or 12
    drive.feed();
  }

  // Takes in speed setpoints,convert them to volts and drive robot
  public void moveVolts(double LeftVolts, double RightVolts) {
    rightSide.setVoltage(-RightVolts);
    leftSide.setVoltage(LeftVolts);
    drive.feed();
  }

  // Takes in speed setpoints and drive robot
  public void moveBase(double LeftSpeed, double RightSpeed) {
    drive.tankDrive(LeftSpeed, -RightSpeed);
  }

  public double getGyroAngle() {
    return ahrs.getAngle();
  }

  public Pose2d getPose() {
    Pose2d temp = odometry.getPoseMeters();
    return temp;
  }

  // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
  // travel distance. gear ratio A:1 means, 1/A.
  public double ticksToPosition(double ticks, double wheelDiameter, double gearRatio) {
    double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
    double nbTurnWheel = nbTurnMotor * gearRatio;
    double distanceTravel = nbTurnWheel * Math.PI * wheelDiameter;
    return distanceTravel;
  }
  // calculating motor + wheel rotations
  // private int velocityToNativeUnits(double velocityMetersPerSecond, double
  // wheelDiameter, double gearRatio) {
  // double wheelRotationsPerSecond = velocityMetersPerSecond / (Math.PI *
  // wheelDiameter);
  // double motorRotationsPerSecond = wheelRotationsPerSecond * gearRatio;
  // double motorRotationsPer100ms = motorRotationsPerSecond /
  // Constants.k100msPerSecond;
  // int sensorTicksPer100ms = (int) (motorRotationsPer100ms *
  // Constants.encoderTicksPerTurn);
  // return sensorTicksPer100ms;
  // }

  private double NativeUnitsToVelocity(double sensorTicksPer100ms, double wheelDiameter, double gearRatio) {
    double motorRotationsPer100ms = (double) (sensorTicksPer100ms / Constants.encoderTicksPerTurn);
    double motorRotationsPerSecond = motorRotationsPer100ms * Constants.k100msPerSecond;
    double wheelRotationsPerSecond = motorRotationsPerSecond / gearRatio;
    double velocityMetersPerSecond = wheelRotationsPerSecond * (Math.PI * wheelDiameter);
    return velocityMetersPerSecond;
  }

  ;

  // Move the robot forward for a set distance at a set speed
  public void DriveStraight(double autoDriveDistance, double autoDriveSpeed) {
    // If the encoder is >= to the distance stop the robot
    if (DriveEncoders.rfEncoderValue >= autoDriveDistance) {
      stopDriveTrain();
      System.out.println(DriveEncoders.rfEncoderValue);
    }
    // Move forward at the set speed
    else {
      move(autoDriveSpeed, autoDriveSpeed);
    }
  }

  // Turn the robot a specified number of degrees left or right
  // + = Right Turn; - = Left Turn
  public void RotateDegrees(double targetDegrees) {
    double KpAim = -0.1;
    double min_aim_command = 0.05;

    double tx = ahrs.getAngle() + targetDegrees;

    double heading_error = -tx;
    // double distance_error = -ty;
    double steering_adjust = 0.0f;

    if (tx > 1.0) {
      steering_adjust = KpAim * heading_error - min_aim_command;
    } else if (tx < 1.0) {
      steering_adjust = KpAim * heading_error + min_aim_command;
    }

    // double distance_adjust = KpDistance * distance_error;

    double left_command = steering_adjust;
    double right_command = steering_adjust * -1;
    drive.tankDrive(left_command, right_command);

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        NativeUnitsToVelocity(leftFront.getSelectedSensorVelocity(), Constants.wheelDiameter,
            Constants.driveTrainGearRatio),
        NativeUnitsToVelocity(rightFront.getSelectedSensorVelocity(), Constants.wheelDiameter,
            Constants.driveTrainGearRatio));
  }

  public boolean checkCalibrationStatus() {
    return ahrs.isCalibrating();
  }

  public void AimingUsingVision() {
    float KpAim = -0.1f;
    // float KpDistance = -0.1f;
    float min_aim_command = 0.05f;

    double tx = Limelight.getLimeLightX();
    // double ty = Limelight.getLimeLightY();

    {
      double heading_error = -tx;
      // double distance_error = -ty;
      double steering_adjust = 0.0f;

      if (tx > 1.0) {
        steering_adjust = KpAim * heading_error - min_aim_command;
      } else if (tx < 1.0) {
        steering_adjust = KpAim * heading_error + min_aim_command;
      }

      // double distance_adjust = KpDistance * distance_error;

      double left_command = steering_adjust;
      double right_command = steering_adjust * -1;
      drive.tankDrive(left_command, right_command);

    }

  }

  public void resetEncoders() {
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }
}
