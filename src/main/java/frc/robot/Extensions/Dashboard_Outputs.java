// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Constants;

/** Methods to get values off of the network tables/shuffleboard */
public class Dashboard_Outputs {
    
    // Get Max speed value
    public double getMaxSpeed(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        double maxSpeed = table.getEntry("Max Speed").getDouble(Constants.speedLimitDefault);
        return maxSpeed;
    }
    public boolean getDriveType(){
        NetworkTable table=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        boolean driveType = table.getEntry("Drive Type = Tank").getBoolean(Constants.arcadeDrive);
        return driveType;
    }
	public double getshooterTargetRPM() {
        NetworkTable table=NetworkTableInstance.getDefault().getTable("Shuffleboard/Inputs");
        double shooterTargetRpm = table.getEntry("Shooter Target RPM").getDouble(Constants.targetRPM);
		return shooterTargetRpm;
	} 


}