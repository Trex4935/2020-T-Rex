// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Magazine;

// Use the intake smacna as a "button"
public class IntakeTrigger extends Trigger {

  // Override the trigger get method
  @Override
  public boolean get() {
    // Pull the state of the intake sensor
    return Magazine.getMagazineSensor();
  }
}
