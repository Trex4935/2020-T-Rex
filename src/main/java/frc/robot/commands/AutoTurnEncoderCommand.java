// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoTurnEncoderCommand extends CommandBase {
  private final DriveTrain driveTrain;
  private double l_turnDistance;
  private int l_turnDirection;
  private boolean finished;

  /** Creates a new AutoTurnEncoderCommand. */
  public AutoTurnEncoderCommand(DriveTrain dt, double turnDistance, int turnDirection) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    l_turnDistance = turnDistance;
    l_turnDirection = turnDirection;
    addRequirements(driveTrain);
        finished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveTrain.RotateEncoder(l_turnDistance, l_turnDirection)) {
     end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopDriveTrain();
    finished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
