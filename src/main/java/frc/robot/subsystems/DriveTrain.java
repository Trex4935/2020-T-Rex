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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TrajectoryContainer;
import frc.robot.Extensions.Dashboard_Outputs;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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

  // Drive Type
  Dashboard_Outputs dashOut;

  // Trajectory
  public Trajectory trajectory;
  public Trajectory trajectoryGSearch;
  public Trajectory trajectorySlalom;
  public Trajectory trajectoryBarrelRacing;
  public Trajectory trajectoryBounce;
  public Trajectory trajectoryShootTrow;  
  public Trajectory trajectoryLineForward;
  public Trajectory trajectoryLineBackward;
  public Trajectory trajectoryBounce1;
  public Trajectory trajectoryBounce2;
  public Trajectory trajectoryBounce3;
  public Trajectory trajectoryBounce4;

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
    //leftFront.configOpenloopRamp(Constants.openLoopRamp);

    rightFront = new WPI_TalonFX(Constants.rightFrontCanID);
    rightFront.setInverted(Constants.inversion);
    //rightFront.configOpenloopRamp(Constants.openLoopRamp);

    leftRear = new WPI_TalonFX(Constants.leftRearCanID);
    leftRear.setInverted(Constants.inversion);
    //leftRear.configOpenloopRamp(Constants.openLoopRamp);

    rightRear = new WPI_TalonFX(Constants.rightRearCanID);
    rightRear.setInverted(Constants.inversion);
    //rightRear.configOpenloopRamp(Constants.openLoopRamp);

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

    // Setup the dashboard input object
    dashOut = new Dashboard_Outputs();

    // Odometry
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-getGyroAngle()), Constants.startPosition);
    System.out.println(ahrs.isCalibrating());
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
    String trajectoryJSONSim = Constants.pathSim + "Slalom.wpilib.json";
    String trajectoryJSONRobot = Constants.pathRobot + "Bounce27.1.wpilib.json";//SLOWBarrelRacing.wpilib.json
    trajectory = TrajectoryContainer.makeTrajectory(trajectoryJSONRobot);
    //Autonomous Trajectory
      // Galactic search 
    trajectoryGSearch = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "GSearch.wpilib.json");
      // Auto Nav
    trajectorySlalom = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "SlowSlalomReal3.27.wpilib.json");
    trajectoryBarrelRacing = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "SLOWBarrelRacingReal27.wpilib.json");
    trajectoryBounce = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Bounce27.4.wpilib.json");
      // Shooter Trow
    trajectoryShootTrow = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Slalom.wpilib.json");
    trajectoryLineForward = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Straight.wpilib.json");
    trajectoryLineBackward = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Backward.wpilib.json");
      // Bounce Path      
    trajectoryBounce1 = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Bounce27.1.wpilib.json");
    trajectoryBounce2 = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Bounce27.2.wpilib.json");
    trajectoryBounce3 = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Bounce27.3.wpilib.json");
    trajectoryBounce4 = TrajectoryContainer.makeTrajectory(Constants.pathRobot + "Bounce27.4.wpilib.json");
    time = 0.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(ahrs.getRotation2d(),
        ticksToPosition(leftFront.getSelectedSensorPosition(), Constants.wheelDiameter, Constants.driveTrainGearRatio),
        ticksToPosition(-rightFront.getSelectedSensorPosition(), Constants.wheelDiameter,
            Constants.driveTrainGearRatio));
     //System.out.println(leftFront.getSelectedSensorPosition());
     //System.out.println(rightFront.getSelectedSensorPosition());
    // System.out.println(ahrs.getRotation2d());
    //System.out.println(odometry.getPoseMeters());
    // System.out.println(odometry.update(ahrs.getRotation2d(),
    // ticksToPosition(leftFront.getSelectedSensorPosition(),
    // Constants.wheelDiameter, Constants.driveTrainGearRatio) ,
    // ticksToPosition(rightFront.getSelectedSensorPosition(),
    // Constants.wheelDiameter, Constants.driveTrainGearRatio)));
    //System.out.println(odometry);
    //System.out.println(getGyroAngle());
    // System.out.println(ticksToPosition(rightFront.getSelectedSensorPosition(),
     //Constants.wheelDiameter, Constants.driveTrainGearRatio));
    // System.out.println(ticksToPosition(leftFront.getSelectedSensorPosition(),
    // Constants.wheelDiameter, Constants.driveTrainGearRatio));
    driveEncoders.SetDriveEncoders(leftFront.getSelectedSensorPosition(), leftRear.getSelectedSensorPosition(),
        rightFront.getSelectedSensorPosition(), rightRear.getSelectedSensorPosition());
  }

  // Method to control the drive with the controller
  // controller = Input controller
  // speedLimiter = value to limit the speed of the motors
  // if else statement to swap between arcade and tank
  public void driveWithController(XboxController controller, double speedLimiter) {
    drive.tankDrive(controller.getRawAxis(Constants.rightTankAxis) * speedLimiter * -1,
       controller.getRawAxis(Constants.leftTankAxis) * speedLimiter * -1);

}

// Duplicate the right controller output onto left and right tank drive
public void driveStraitWithController(XboxController controller,double speedLimiter){
 double rightSpeed = controller.getRawAxis(Constants.rightTankAxis) * speedLimiter * -1;
 drive.tankDrive(rightSpeed,rightSpeed);
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

  public void resetGyro(){
    ahrs.reset();
    //ahrs.zeroYaw();
  }

  // Method to just stop the drive
  public void stopDriveTrain() {
    drive.stopMotor();
  }

  // Print out way points
  public void getWP(double time) {
    // System.out.println(time);
    // System.out.println(trajectory.sample(time));
  }

  public Pose2d getPosition() {
    time += 0.02;
    System.out.println(trajectory.sample(time).poseMeters);
    System.out.println(trajectory.sample(time).velocityMetersPerSecond);
    Pose2d temp = getPose();
    return trajectory.sample(time).poseMeters;
  }

  // Takes in speed setpoints,convert them to volts and drive robot
  public void move(double LeftSpeed, double RightSpeed) {
    
    rightSide.setVoltage(-(LeftSpeed / Constants.kvVoltSecondsPerMeter) * Constants.autoCalibrate * (1)); // Or 12 or
                                                                                                           // kvVoltSecondsPerMeter
                                                                                                           // *WheelRatio
    leftSide.setVoltage((RightSpeed / Constants.kvVoltSecondsPerMeter) * Constants.autoCalibrate * (1)); // Or 12
    drive.feed();
    // System.out.println(RightSpeed);
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
   // System.out.println(temp);
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

  private int velocityToNativeUnits(double velocityMetersPerSecond, double wheelDiameter, double gearRatio) {
    double wheelRotationsPerSecond = velocityMetersPerSecond / (Math.PI * wheelDiameter);
    double motorRotationsPerSecond = wheelRotationsPerSecond * gearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / Constants.k100msPerSecond;
    int sensorTicksPer100ms = (int) (motorRotationsPer100ms * Constants.encoderTicksPerTurn);
    return sensorTicksPer100ms;
  }

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
    }
    // Move forward at the set speed
    else {
      move(autoDriveSpeed, autoDriveSpeed);
    }
  }

  // Turn the robot a specified number of degrees left or right
  // + = Right Turn; - = Left Turn
  public void RotateDegrees(double degrees) {
    double leftspeed = Constants.rotationSpeed;
    double rightspeed = Constants.rotationSpeed;

    // If zero then we aren't going to move so set the speed to zero
    if (degrees == 0) {
      leftspeed = 0;
      rightspeed = 0;
      // if negative we are turning left so set the left to run backwards
    } else if (degrees < 0) {
      leftspeed = leftspeed * -1;
      // if positive we are turning right so set the right to run backwards
    } else {
      rightspeed = rightspeed * -1;
    }

    

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
