// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

/** Add your docs here. */
// Get values from Shuffleboard widgets to use in running code
public class Dashboard_Outputs {
    public double getMaxSpeed(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        double maxSpeed = table.getEntry("Max Speed").getDouble(Constants.speedLimitDefault);
        return maxSpeed;
    }
    public boolean getDriveType(){
        NetworkTable table2=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        boolean driveType = table2.getEntry("Drive Type = Tank").getBoolean(Constants.arcadeDrive);
        return driveType;
    } 
}