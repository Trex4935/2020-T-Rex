// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Belt drive motor
// Intake drve motor
// Sensors

/// Actions:
// Pull in ball
// Spit out ball

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {

  /// Motors
  WPI_TalonSRX beltMotor;
  WPI_TalonSRX intakeMotor;
  WPI_TalonSRX reverseIntakeMotor;

  /// Smacna
  DigitalInput Smacna; 

  /** Creates a new Magazine. */
  public Magazine() {

    beltMotor = new WPI_TalonSRX(Constants.beltMotorCanID);
    beltMotor.setInverted(false);

    intakeMotor = new WPI_TalonSRX(Constants.intakeMotorCanID);
    intakeMotor.setInverted(false);

    Smacna = new DigitalInput(Constants.smaknaLocation); 

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // Make belt motor go wrrrr
  public void spitBall() {
    beltMotor.set(Constants.beltMotorSpeed);
  }

  // Run both
  public void runBothMotors() {
    intakeBall();
    spitBall();
  }

  // Make intake motor also go wrrrr
  public void intakeBall() {
    intakeMotor.set(Constants.intakeMotorSpeed);

  }

  // Method to just stop the drive
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  // method to stop the belt motor
  public void stopSpitBall() {
    beltMotor.stopMotor();
  }

  // reverse intake motor
  public void releaseBall() {
    intakeMotor.set(-Constants.intakeMotorSpeed);
    beltMotor.set(-Constants.beltMotorSpeed);
  }

  // getting a smacna value and
  public boolean getInput() {
    boolean a = Smacna.get();
    return (a);
  }
}
