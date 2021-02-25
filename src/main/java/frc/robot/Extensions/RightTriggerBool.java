// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// Using the right trigger as a button
public class RightTriggerBool extends Trigger {

    // Overriding the trigger get method
    @Override
    public boolean get() {
        // getRawAxis returns 0-1 on xbox controller so we are going to define a lower bound of 0.25
        // Anything over that is considered Pressed
        if (RobotContainer.controller.getRawAxis(Constants.rtTrigger) >= 0.25) {
            // Trigger is pressed
            return true;
        } else {
            // Trigger is NOT pressed
            return false;
        }
    }
}
