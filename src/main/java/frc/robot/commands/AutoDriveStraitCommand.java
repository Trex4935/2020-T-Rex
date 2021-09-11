// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoDriveStraitCommand extends CommandBase {
  private final DriveTrain driveTrain;
  private double l_autoDriveDistance;
  private double l_autoDriveSpeed;

  /** Creates a new AutoForward. */
  public AutoDriveStraitCommand(DriveTrain dt,double autoDriveDistance, double autoDriveSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    l_autoDriveDistance = autoDriveDistance;
    l_autoDriveSpeed = autoDriveSpeed;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.DriveStraight(l_autoDriveDistance, l_autoDriveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDriveTrain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
