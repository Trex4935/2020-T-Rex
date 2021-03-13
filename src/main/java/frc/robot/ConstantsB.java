// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
// import frc.robot.Extensions.Gains;

// /**
//  * The Constants class provides a convenient place for teams to hold robot-wide
//  * numerical or boolean constants. This class should not be used for any other
//  * purpose. All constants should be declared globally (i.e. public static). Do
//  * not put anything functional in this class.
//  *
//  * <p>
//  * It is advised to statically import this class (or one of its inner classes)
//  * wherever the constants are needed, to reduce verbosity.
//  */
// public final class Constants {

//     // region CAN IDS
//     // ================================

//     // Drivetrain motors
//     public static final int leftFrontCanID = 11;
//     public static final int rightFrontCanID = 12;
//     public static final int leftRearCanID = 13;
//     public static final int rightRearCanID = 14;

//     // Magazine motors
//     public static final int beltMotorCanID = 6;
//     public static final int intakeMotorCanID = 2;

//     // Shooter motors
//     public static final int shooterMotorID = 21;

//     // Elevator Motors
//     // Not currently implemented
//     // endregion

//     // region Magazine
//     // ================================

//     // Magazine motor speed
//     public static final double beltMotorSpeed = 0.6;
//     public static final double intakeMotorSpeed = 0.6;

//     // Smacna DIO location on the roborio
//     public static final int intakeSensorDIO = 1;
//     public static final int magazineSensorDIO = 2;
//     public static final int shooterSensorDIO = 3;

//     // How long we run the mag when we sense a ball
//     public static final double intakeTimeOut = 1;

//     // endregion

//     // region Drivetrain
//     // ================================

//     // Controller axis values for controlling the drive
//     public static final int leftTankAxis = 5;
//     public static final int rightTankAxis = 1;
//     public static final int rightArcadeAxis = 4;

//     // Used to limit speed of the robot
//     public static final double speedLimitDefault = 0.5;

//     // Changes from tank drive to arcade drive
//     public static final boolean arcadeDrive = false;

//     // endregion

//     // region Shooter
//     // ================================

//     // Shooter motor speed
//     public static final double shooterSpeed = 0.5;
//     public static final int rtTrigger = 3;
//     public static final int ltTrigger = 2;

//     // PID stuff
//     public static final int kSlotIdx = 0;
//     public static final int kPIDLoopIdx = 0;
//     public static final int kTimeoutMs = 30;

//     // Pid values                                                 kP    kI   kD        kF          Iz  PeakOut
//     public final static Gains kGains_Velocit_Shooter = new Gains(0.74, 0.001, 5, 1023.0 / 20660.0, 300, 1.00);

//     // Target RPM value for shooter motor
//     public static final int targetRPM = 5000;

//     // endregion

//     // region Controller
//     // ================================

//     // USB port as shown in the dashboard that the xbox controller is connected to
//     public static final int xboxControllerPort = 0;

//     // endregion

//     // region Autonomous
//     // ================================

//     // How fast we move forward in auto
//     public static final double autoLeftSpeed = 0.9;
//     public static final double autoRightSpeed = 0.9;

//     // How long we move forward
//     public static final double secondForward = 10;

//     // Trajectory 
    
// 	public static final double ksVolts = 0.691;
// 	public static final double kvVoltSecondsPerMeter = 0.141*(Math.PI*0.1524);
//     public static final double kaVoltSecondsSquaredPerMeter = 0.0262*(Math.PI*0.1524)*(Math.PI*0.1524);
//     public static final double kPDriveVel = 8.5;//1.15;
// 	public static final double kMaxSpeedMetersPerSecond = 3;
//     public static final double kMaxAccelerationMetersPerSecondSquared = 3;
//     public static final double kTrackwidthMeters = 0.582;
// 	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
// 	public static final double kRamseteB = 2;
// 	public static final double kRamseteZeta = 0.7;
//     //Field
//     public static final Pose2d startPosition = new Pose2d(0, 0, new Rotation2d());
//     //Dynamics
//     public static final double wheelDiameter = 0.1524; //m, 6inch
// 	public static final double driveTrainGearRatio = 1/30;

//     //Encoder
// 	public static final double encoderTicksPerTurn = 2048;
//     // endregion
// 	public static final double k100msPerSecond = 100;

// }