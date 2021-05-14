// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BouncePathCommand extends SequentialCommandGroup {
    //// private final DriveTrain driveTrain;
    /** Creates a new AutonomousAndMagazineCommand. */
    
    public BouncePathCommand(DriveTrain dt) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //replace dt::getPose with driveTrain::getPosition to use simulated encoder and gyro data for simulation and debugging
    addCommands(
      new RamseteCommand(dt.trajectoryBounce1, dt::getPose, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics, dt::move, dt),
      new RamseteCommand(dt.trajectoryBounce2, dt::getPose, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics, dt::move, dt),
      new RamseteCommand(dt.trajectoryBounce3, dt::getPose, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics, dt::move, dt),
      new RamseteCommand(dt.trajectoryBounce4, dt::getPose, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics, dt::move, dt)
    );
  }
}