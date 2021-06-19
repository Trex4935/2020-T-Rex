// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

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
public final class TakeHomePath {
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
}
