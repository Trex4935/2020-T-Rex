// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

/** Add your docs here. */
public class Init_Dashboard {
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
    public NetworkTableEntry maxSpeed =
        tab.add("Max Speed", Constants.speedLimitDefault)
           .getEntry();
}
