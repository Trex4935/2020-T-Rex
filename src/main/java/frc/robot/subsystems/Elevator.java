// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

public class Elevator extends SubsystemBase {

  /*
   * Elevator Components: Motor for the Chain >> Move up and Down Limit Switch
   */

  /// Solenoid
  public static WPI_TalonSRX elevatorSolenoid;

  /// Motor
  public static WPI_TalonSRX elevatorMotor;

  /// Limit switch
  private static DigitalInput topLimitSwitch;
  private static DigitalInput bottomLimitSwitch;

  /** Creates a new Elevator. */
  public Elevator() {

    // initialize the elevator motor
    elevatorMotor = new WPI_TalonSRX(Constants.elevatorMotorCanID);
    elevatorMotor.setInverted(false);

    // initialize the solenoid
    elevatorSolenoid = new WPI_TalonSRX(Constants.elevatorSolenoidCanID);
    elevatorSolenoid.setInverted(false);

    // initialize the limit switches
    topLimitSwitch = new DigitalInput(Constants.elevatorLimitorTopDIO);
    bottomLimitSwitch = new DigitalInput(Constants.elevatorLimitorBottomDIO);

  }

  // Move the elevator at constant speed
  //    if ((Timer.getMatchTime() >= 45) & (topLimitSwitch.get() == true)) {
  public void moveElevatorUp() {
    if (topLimitSwitch.get() == true) {
      elevatorMotor.set(Constants.elevatorMotorSpeed);
    } else {
      stopElevator();
    }
  }

  // Move the solenoid at constant speed
  public void openSolenoid() {
    elevatorSolenoid.set(1);
  }

  // Moves the elevator up or down based off of the controller inputs you press
  // (back or start)

  public void moveElevator(boolean up) {
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

  // Stop the solenoid
  public void stopSolenoid() {
    elevatorSolenoid.stopMotor();
  }

  // Reverse elevator direction and make it go down
  public void moveElevatorDown() {
    if (bottomLimitSwitch.get() == false) {
      stopElevator();
    } else {
      elevatorMotor.set(-Constants.elevatorMotorSpeed);

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void moveElevator(boolean direction){}

  public void lockElevator() {
  }

}
