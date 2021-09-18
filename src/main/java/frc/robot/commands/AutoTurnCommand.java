// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Extensions.DriveEncoders;

public class AutoTurnCommand extends CommandBase {
  /** Creates a new AutoTurnCommand. */
  public AutoTurnCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){}
      public void RotateEncoder(double turnDistance,int direction){

    double rotateSpeed = 0.4;
    double rightSpeed = rotateSpeed;
    double leftSpeed = rotateSpeed;
    
    // Set direction
    if (direction < 0){
      rightSpeed = rightSpeed * -1;  
    }
    else {
      leftSpeed = leftSpeed * -1;
    }

    // While the encoder reads < than our turn value keep turning
    if (DriveEncoders.rfEncoderValue >= turnDistance) {
      stopDriveTrain();
    }
    // Turn at the set speed
    else {
      move(leftSpeed, rightSpeed);
    } 
  }


  private void move(double leftSpeed, double rightSpeed) {
  }

  private void stopDriveTrain() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stopDriveTrain();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
