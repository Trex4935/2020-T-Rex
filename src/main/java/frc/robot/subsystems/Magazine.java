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
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {

  /// Motors
  public WPI_TalonSRX highBeltMotor;
  WPI_TalonSRX lowBeltMotor;
  WPI_TalonSRX reverseIntakeMotor;

  /// Smacna
  private static DigitalInput intakeSensor;
  private static DigitalInput magazineSensor;
  private static DigitalInput shooterSensor;

  /** Creates a new Magazine. */
  public Magazine() {

    highBeltMotor = new WPI_TalonSRX(Constants.beltMotorCanID);
    highBeltMotor.setInverted(true);

    lowBeltMotor = new WPI_TalonSRX(Constants.intakeMotorCanID);
    lowBeltMotor.setInverted(true);

    intakeSensor = new DigitalInput(Constants.intakeSensorDIO);
    magazineSensor = new DigitalInput(Constants.magazineSensorDIO);
    shooterSensor = new DigitalInput(Constants.shooterSensorDIO);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /// Intake system consists of two motors
  /// One drives the highbelt from bend -> Shooter
  /// The other drives the low belt from intake port -> bend
  /// The Bend is the tigest point in the magazine run that bends the ball from
  /// horizontal movement to vertical movement

  /// Designating the belts as HIGHBELT (HB) & LOWBELT (LB)
  /// Designating the motors as HIGHMOTOR (HM) and LOWMOTOR (LM)

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

  // Get the value of the intake sensor
  /*public static boolean getIntakeSensor() {
    boolean a = intakeSensor.get();
    return (!a);
  }
  */
  
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

  // Singulation first iteration
  public void oneBall() {
    if (getMagazineSensor()) {
      moveHighBelt();
    }
  }
}
