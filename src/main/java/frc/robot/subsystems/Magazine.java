// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Belt drive motor
// Intake drive motor
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
  public static WPI_TalonSRX highBeltMotor;
  WPI_TalonSRX lowBeltMotor;
  WPI_TalonSRX reverseIntakeMotor;

  /// Smacna
  private static DigitalInput magazineSensor;
  private static DigitalInput shooterSensor;

  /** Creates a new Magazine. */
  public Magazine() {

    // Declare high belt
    highBeltMotor = new WPI_TalonSRX(Constants.beltMotorCanID);
    highBeltMotor.setInverted(false);

    // Declare low belt
    lowBeltMotor = new WPI_TalonSRX(Constants.intakeMotorCanID);
    lowBeltMotor.setInverted(false);

    // Create sensor objects
    magazineSensor = new DigitalInput(Constants.magazineSensorDIO);
    shooterSensor = new DigitalInput(Constants.shooterSensorDIO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /// Intake system consists of two motors
  /// One drives the highBelt from 25% into the magazine to the shooter
  /// The other drives the low belt from intake port -> 25% point
  /// The Bend is the tightest point in the magazine run that bends the ball from
  /// horizontal movement to vertical movement

  /// Designating the belts as highBelt (HB) & lowBelt (LB)
  /// Designating the motors as highMotor (HM) and lowMotor (LM)

  // Move the HB at a constant speed
  public void moveHighBelt() {
    highBeltMotor.set(Constants.beltMotorSpeed);
  }

  // Move the LB at a constant speed
  public void moveLowBelt() {
    lowBeltMotor.set(Constants.intakeMotorSpeed);
  }

  // Run both belts at a constant speed
  public void moveBothBelts() {
    moveLowBelt();
    moveHighBelt();
  }

  // Stop the LB
  public void stopLowBelt() {
    lowBeltMotor.stopMotor();
  }

  // Stop the HB
  public void stopHighBelt() {
    highBeltMotor.stopMotor();
  }

  // Stop both belts
  public void stopBothBelts() {
    stopLowBelt();
    stopHighBelt();
  }

  // Run the belts backwards to push the balls back out of the intake channel
  public void releaseBall() {
    lowBeltMotor.set(-Constants.intakeMotorSpeed);
    highBeltMotor.set(-Constants.beltMotorSpeed);
  }

  // Get the value of the magazine sensor
  public static boolean getMagazineSensor() {
    boolean a = magazineSensor.get();
    return (!a);
  }

  // Get the value of the shooter sensor
  public static boolean getShooterSensor() {
    boolean a = shooterSensor.get();
    return (!a);
  }

  // Singulation of the ball when intake mode is turned on
  // Intake mode turns on the LB via the LM
  public void singulateBall() {
    // When the magazine sensor sees a ball run the HB
    if (getMagazineSensor()) {
      moveHighBelt();
      // Constants.actualBallCount
    }
    // When it no longer sees a ball then it is "in" the magazine so we stop the
    // motor
    else {
      stopHighBelt();
    }
  }

  public void emptyMagToShooter(boolean atSpeed) {
    if (atSpeed) {
      moveBothBelts();
    } else {
      stopBothBelts();
    }
  }
}
