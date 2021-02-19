// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveWithWPCommand extends CommandBase {
  DriveTrain drivetrain;
  Timer timer;
  Boolean printStop;
  /** Creates a new DriveWithWPCommand. */
  public DriveWithWPCommand(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = dt;
    timer = new Timer();
    printStop = false;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    while(timer.get() < 5) {
      drivetrain.getWP(timer.get());
    }
    timer.stop();
    printStop = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return printStop;
  }
}
