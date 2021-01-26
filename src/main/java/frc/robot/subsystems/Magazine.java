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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {

  /// Motors
  WPI_TalonSRX beltMotor;
  WPI_TalonSRX intakeMotor;

  /** Creates a new Magazine. */
  public Magazine() {

    beltMotor = new WPI_TalonSRX(Constants.beltMotorCanID);
    beltMotor.setInverted(false);

    intakeMotor = new WPI_TalonSRX(Constants.intakeMotorCanID);
    intakeMotor.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  // Make belt motor go wrrrr
  public void spitBall() {
    beltMotor.set(ControlMode.PercentOutput, Constants.beltMotorSpeed);
  }

  // Make intake motor also go wrrrr
  public void intakeBall() {
    //intakeMotor.set(ControlMode.PercentOutput, Constants.intakeMotorSpeed);
    intakeMotor.set(0.5);
  }

  // Method to just stop the drive
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

}
