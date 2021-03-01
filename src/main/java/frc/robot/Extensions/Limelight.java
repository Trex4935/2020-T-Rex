// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


//Creating a class for the Limelight data/
public class Limelight {

// Reading Limelight values from NetworkTables and creating objects to re-use
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");

// Limelight X axis value read into object
public double getLimeLightX(){
double limeLightX = tx.getDouble(0.0);
return limeLightX;
}

// Limelight Y axis value read into object
public double getLimeLightY(){
double limeLightY = ty.getDouble(0.0);
return limeLightY;
}

// Limelight Area value read into object
public double getLimeLightArea(){
double limeLightArea = ta.getDouble(0.0);
return limeLightArea;
}

}
