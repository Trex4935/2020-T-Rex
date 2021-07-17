// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.robot.Extensions.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    // region CAN IDS
    // ================================

    // Drivetrain motors
    public static final int leftFrontCanID = 12;
    public static final int rightFrontCanID = 11;
    public static final int leftRearCanID = 14;
    public static final int rightRearCanID = 13;

    // Magazine motors
    public static final int beltMotorCanID = 3;
    public static final int intakeMotorCanID = 2;

    // Shooter motors
    public static final int shooterMotorID = 21;

    // Elevator Motors
    public static final int elevatorMotorCanID = 5;

    // Elevator Solenoid
    public static final int elevatorSolenoidCanID = 4;

    // endregion

    // region Magazine
    // ================================

    // Magazine motor speed
    public static final double beltMotorSpeed = 0.55;
    public static final double intakeMotorSpeed = 0.5;

    public static int ballCount = 0;
    public static final int actualBallCount = ballCount++;

    // Smacna DIO location on the roborio
    public static final int magazineSensorDIO = 2;
    public static final int shooterSensorDIO = 3;

    // endregion

    // region Drivetrain
    // ================================

    // Controller axis values for controlling the drive
    public static final int leftTankAxis = 1;
    public static final int rightTankAxis = 5;
    public static final int rightArcadeAxis = 4;

    // Used to limit speed of the robot
    public static final double speedLimitDefault = 0.75;

    // Changes from tank drive to arcade drive
    public static final boolean arcadeDrive = false;

    // Open loop ramp value to keep the motors going from 0-100
    public static final double openLoopRamp = 0.2;

    // Change the what direction is the front of the robot
    public static final int driveDirection = -1;

    // Change the inversion of the robot
    public static final boolean inversion = true;

    // Basic autonomous stuff
    public static final double rotationSpeed = 0.4;

    // endregion

    // region Shooter
    // ================================

    // Shooter motor speed
    public static final double shooterSpeed = 0.3;
    public static final int rtTrigger = 3;
    public static final int ltTrigger = 2;
    public static boolean atSpeed = false;

    // PID stuff
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    public static final int PIDRange = 100;

    // Pid values kP kI kD kF Iz PeakOut
    // public final static Gains kGains_Velocity_Shooter = new Gains(0.74, 0.001, 5, 1023.0 / 20660.0, 300, 1.00);
    public final static Gains kGains_Velocity_Shooter = new Gains(0.08, 0.0, 0, 0.055, 300, 1.00);

    // Target RPM value for shooter motor
    public static final int targetRPM = 4000;

    // endregion

    // region Controller
    // ================================

    // USB port as shown in the dashboard that the xbox controller is connected to
    public static final int xboxControllerPort = 0;

    // endregion

    // region Autonomous
    // ================================

    // How fast we move forward in auto
    public static final double autoLeftSpeed = 0.9;
    public static final double autoRightSpeed = 0.9;

    // How long we move forward
    public static final double secondForward = 1;

    // Trajectory
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kTrackWidthMeters = 0.584; // 0.584 testing> .65
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackWidthMeters);
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    // Field
    public static final Pose2d startPosition = new Pose2d(0, 0, new Rotation2d());

    // Dynamics
    public static final double wheelDiameter = 0.1524; // m, 6inch
    public static final double driveTrainGearRatio = 1.00 / 9.52; // 1.00/30;1.00/9.56

    // Encoder
    public static final double encoderTicksPerTurn = 2048;
    public static final double k100msPerSecond = 100;
    public static final double autoCalibrate = 6 * 1.32 * 0.95;

    // Path Strings
    public static final String pathSim = "..\\.\\deploy\\paths\\";
    public static final String pathRobot = "/home/lvuser/deploy/paths/";
	

    // endregion

    // region Elevator
    // ================================
    public static final double elevatorMotorSpeed = 0.6;

    // Elevator Sensors DIO Locations on the Roborio 
    public static final int elevatorLimitorTopDIO = 0;
    public static final int elevatorLimitorBottomDIO = 1; 

    // endregion
}