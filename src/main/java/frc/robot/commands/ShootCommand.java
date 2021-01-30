// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
  /** Creates a new Shoot. */
  private final Shooter shooter;
  
  public ShootCommand(Shooter sh) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = sh;
    addRequirements(shooter);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shoot();
    // System.out.println(RobotContainer.controller.getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shootStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
