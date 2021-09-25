// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.TrajectoryRepo;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousTripleShootSimpleCommand extends SequentialCommandGroup  {
  ////private final DriveTrain driveTrain;
  /** Creates a new AutonomousAndMagazineCommand. */
  public AutonomousTripleShootSimpleCommand( DriveTrain dt, Magazine mag, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //replace dt::getPose with driveTrain::getPosition to use simulated encoder and gyro data for simulation and debugging
    addCommands(
      new EmptyMagToShooterCommand(mag).alongWith(new ShootPIDCommand(shooter, Constants.targetRPM)).withTimeout(4),
      new AutoDriveStraitCommand(dt,45000,-1.0),
      new AutoTurnEncoderCommand(dt, 7500, -1),
      new AutoDriveStraitCommand(dt,140000,-0.6).raceWith(new SingulateBallCommand(mag)),
      new AutoDriveStraitCommand(dt,100,1.0),
      new AutoTurnEncoderCommand(dt, 5500, 1),
      new EmptyMagToShooterCommand(mag).alongWith(new ShootPIDCommand(shooter, 3500)).withTimeout(4)
      //new RamseteCommand(TrajectoryRepo.trajectoryBack, dt::getPose, new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), Constants.kDriveKinematics, dt::move, dt)
      );
  }
}
