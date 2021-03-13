// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Extensions.Dashboard_Outputs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;

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
  AHRS ahrs;

  // Odometry
  DifferentialDriveOdometry odometry;

  // Drive Type
  Dashboard_Outputs dashOut;

  // Trajectory
  public Trajectory trajectory;

  // Internal time

  public static double time;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    // Setup each of the motors for use later
    // Going to set any whole game settings here as well (like motor inversion)
    leftFront = new WPI_TalonFX(Constants.leftFrontCanID);
    leftFront.setInverted(false);
    leftFront.configOpenloopRamp(Constants.openLoopRamp);

    rightFront = new WPI_TalonFX(Constants.rightFrontCanID);
    rightFront.setInverted(false);
    rightFront.configOpenloopRamp(Constants.openLoopRamp);

    leftRear = new WPI_TalonFX(Constants.leftRearCanID);
    leftRear.setInverted(false);
    leftRear.configOpenloopRamp(Constants.openLoopRamp);

    rightRear = new WPI_TalonFX(Constants.rightRearCanID);
    rightRear.setInverted(false);
    rightRear.configOpenloopRamp(Constants.openLoopRamp);

    ahrs = new AHRS(SPI.Port.kMXP);
    
    // Adding encoders data to the dashboard
    //Shuffleboard.getTab("Driver Info").add("Left Front", leftFront.getSelectedSensorPosition()).withWidget("Text View").withPosition(1,1).withSize(1,1);
    //Shuffleboard.getTab("Driver Info").add("Left Rear", leftRear.getSelectedSensorPosition()).withWidget("Text View").withPosition(2,1).withSize(1,1);
    //Shuffleboard.getTab("Driver Info").add("Right Front", rightFront.getSelectedSensorPosition()).withWidget("Text View").withPosition(1,2).withSize(1,1);
    //Shuffleboard.getTab("Driver Info").add("Right Rear", rightRear.getSelectedSensorPosition()).withWidget("Text View").withPosition(2,2).withSize(1,1);

    // create the speed controller groups for use in the differential drive
    // each one should be a pairing of the motors on a given side of the robot
    rightSide = new SpeedControllerGroup(rightFront, rightRear);
    leftSide = new SpeedControllerGroup(leftFront, leftRear);

    // create the drive object that will control the differential drive
    // It needs both a set of left and right motors
    // file:///C:/Users/Public/wpilib/2021/documentation/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.html
    drive = new DifferentialDrive(rightSide, leftSide);

    dashOut = new Dashboard_Outputs();

    // Odometry
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getGyroAngle()), Constants.startPosition);

    // Trajectory

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    // trajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(
    // new Translation2d(1, 1),
    // new Translation2d(2, -1)
    // ),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // // Pass config
    // config
    // );

    // Doc on how to access the file via the Robo Rio
    // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html
    //String trajectoryJSON = "..\\.\\deploy\\paths\\SlowLine.wpilib.json";
    //String trajectoryJSON = "..\\.\\deploy\\paths\\Line.wpilib.json";
    String trajectoryJSON = "..\\.\\deploy\\paths\\Slalom.wpilib.json";
    //String trajectoryJSON = "/home/lvuser/deploy/paths/Line.wpilib.json";//"..\\.\\deploy\\paths\\Line.wpilib.json"
    trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    time = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(ahrs.getRotation2d(), ticksToPosition(leftFront.getSelectedSensorPosition(), Constants.wheelDiameter, Constants.driveTrainGearRatio) , ticksToPosition(rightFront.getSelectedSensorPosition(), Constants.wheelDiameter, Constants.driveTrainGearRatio));
    System.out.println(leftFront.getSelectedSensorPosition());
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
    }
  }

  // Move us forward during auto
  public void autoForward(double seconds) {
    drive.tankDrive(Constants.autoLeftSpeed, Constants.autoRightSpeed);
  }

  public static void resetTime(){
    time = 0;
  }

  // Method to just stop the drive
  public void stop() {
    drive.stopMotor();
  }

  // Print out way points
  public void getWP(double time) {
    System.out.println(time);
    System.out.println(trajectory.sample(time));
  }

  public Pose2d getPosition() {
    time += 0.02;
    System.out.println(trajectory.sample(time).poseMeters);
    System.out.println(trajectory.sample(time).velocityMetersPerSecond);
    return trajectory.sample(time).poseMeters;
  }

  // Takes in speed setpoints,convert them to volts and drive robot
  public void move(double LeftSpeed, double RightSpeed) {
    rightSide.setVoltage(-RightSpeed / Constants.kvVoltSecondsPerMeter); // Or 12 or kvVoltSecondsPerMeter *WheelRatio
    leftSide.setVoltage(LeftSpeed / Constants.kvVoltSecondsPerMeter); // Or 12
    drive.feed();
    System.out.println(RightSpeed);
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
    return odometry.getPoseMeters();
  }

  // Takes the rotation or internal ticks of Falcon Encoder and turn them to a
  // travel distance. gear ratio A:1 means, 1/A.
  public double ticksToPosition(double ticks, double wheelDiameter, double gearRatio) {
    double nbTurnMotor = ticks / Constants.encoderTicksPerTurn;
    double nbTurnWheel = nbTurnMotor * gearRatio;
    double distanceTravel = nbTurnWheel * Math.PI * wheelDiameter;
    return distanceTravel;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond, double wheelDiameter, double gearRatio){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(Math.PI * wheelDiameter);
    double motorRotationsPerSecond = wheelRotationsPerSecond * gearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / Constants.k100msPerSecond;
    int sensorTicksPer100ms = (int)(motorRotationsPer100ms * Constants.encoderTicksPerTurn);
    return sensorTicksPer100ms;
  }

  private double NativeUnitsToVelocity(double sensorTicksPer100ms, double wheelDiameter, double gearRatio ){
    double motorRotationsPer100ms = (double)(sensorTicksPer100ms / Constants.encoderTicksPerTurn);
    double motorRotationsPerSecond = motorRotationsPer100ms * Constants.k100msPerSecond;
    double wheelRotationsPerSecond = motorRotationsPerSecond  / gearRatio;
    double velocityMetersPerSecond = wheelRotationsPerSecond*(Math.PI * wheelDiameter);
    return velocityMetersPerSecond;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(NativeUnitsToVelocity(leftFront.getSelectedSensorVelocity(), Constants.wheelDiameter, Constants.driveTrainGearRatio), NativeUnitsToVelocity(rightFront.getSelectedSensorVelocity(), Constants.wheelDiameter, Constants.driveTrainGearRatio));
  }

}
