// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

/** Methods to get values off of the network tables and write them to Shuffleboard */
public class Dashboard_Outputs {
    
    // Get Max speed value and write to Shffleboard
    public double getShooterSpeed(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        double ShooterSpeed = table.getEntry("Max Speed").getDouble(Constants.speedLimitDefault);
        return ShooterSpeed;
    }
    // Get the Drive Type value and write to Shffleboard
    public boolean getDriveType(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        boolean driveType = table.getEntry("Drive Type = Tank").getBoolean(Constants.arcadeDrive);
        return driveType;
    }
    // Get the Shooter RPM setting value and write to Shffleboard
	public double getShooterTargetRPM() {
        NetworkTable table=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        double shooterTargetRpm = table.getEntry("Shooter Target RPM").getDouble(Constants.targetRPM);
		return shooterTargetRpm;
	} 


}