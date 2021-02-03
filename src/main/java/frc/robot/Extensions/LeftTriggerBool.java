// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// Class to extend the Trigger class and allow us to move the left trigger to 
public class LeftTriggerBool extends Trigger {

    @Override
    public boolean get() {
        if (RobotContainer.controller.getRawAxis(Constants.ltTrigger) >= 0.25) {
            return true;
        } else {
            return false;
        }
    }
}