// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    // Can IDs for the drivetrain motors
    public static final int leftFrontCanID = 11;
    public static final int rightFrontCanID = 12;
    public static final int leftRearCanID = 13;
    public static final int rightRearCanID = 14;

    // Magazine motors
    public static final int beltMotorCanID = 6;
    public static final int intakeMotorCanID = 2;

    // Magazine motor speed
    public static final double beltMotorSpeed = 0.6;
    public static final double intakeMotorSpeed = 0.6;

    // Controller axis values for controlling the drive
    public static final int leftTankAxis = 1;
    public static final int rightTankAxis = 5;
    public static final int rightArcadeAxis = 4;

    // Used to limit speed of the robot
    public static final double speedLimitDefault = 0.25;

    // USB port as shown in the dashboard that the xbox controller is connected to
    public static final int xboxControllerPort = 0;

    // How fast we move forward in auto
    public static final double autoLeftSpeed = 0.9;
    public static final double autoRightSpeed = 0.9;

    // How long we move forward
    public static final double secondForward = 10;

    // Changes from tank drive to arcade drive
    public static final boolean arcadeDrive = false;

    // Shooter motors
    public static final int shooterMotorID = 21;

    // Shooter motor speed
    public static final double shooterSpeed = 0.5;
    public static final int rtTrigger = 3;
    public static final int ltTrigger = 2;

    // Smacna DIO location on the roborio
    public static final int smaknaLocation = 9;

    // PID stuff 
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 30;
    
    // Pid values
    // 	                                    			  kP   	 kI    kD      kF          Iz    PeakOut 
    public final static Gains kGains_Velocit  = new Gains( 0.1, 0.001, 5, 1023.0/20660.0,  300,  1.00);
    
    // Target RPM value for shooter motor
    public static final int targetRPM = 1000;
}
