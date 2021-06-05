// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  /* Elevator Components:
  Motor for the Chain >> Move up and Down
  Limit Switch
  */

  /// Motor
  public static WPI_TalonSRX elevatorMotor;

  /// Limit switch

  /** Creates a new Elevator. */
  public Elevator() {

    // initialize the elevator motor
    elevatorMotor = new WPI_TalonSRX(Constants.elevatorMotorCanID);
    elevatorMotor.setInverted(false);

  }

  //Move the elevator at constant speed
  public void MoveElevatorUp() {
    elevatorMotor.set(Constants.elevatorMotorSpeed);
  }

  // Moves the elevator up or down based off of the controller inputs you press (back or start)

  public void MoveElevator(boolean up){
    if (up) {
      elevatorMotor.set(Constants.elevatorMotorSpeed);
    } else {
      elevatorMotor.set(-Constants.elevatorMotorSpeed);
    }
  }

  // Stop the elevator
  public void stopElevator() {
    elevatorMotor.stopMotor();
  }

  // Reverse elevator direction and make it go down
  public void MoveElevatorDown() {
    elevatorMotor.set(-Constants.elevatorMotorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveElevator(boolean direction){}

  public void lockElevator(){}

}
