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
static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
static NetworkTableEntry tx = table.getEntry("tx");
static NetworkTableEntry ty = table.getEntry("ty");
static NetworkTableEntry ta = table.getEntry("ta");
static NetworkTableEntry tv = table.getEntry("tv");
static NetworkTableEntry ts = table.getEntry("ts");
static NetworkTableEntry tshort = table.getEntry("tshort");
static NetworkTableEntry tlong = table.getEntry("tlong");
static NetworkTableEntry thor = table.getEntry("thor");
static NetworkTableEntry tvert = table.getEntry("tvert");

// Limelight X axis value read into object
public static double getLimeLightX() {
    double limeLightX = tx.getDouble(0.0);
    return limeLightX;
}

// Limelight Y axis value read into object
public static double getLimeLightY() {
    double limeLightY = ty.getDouble(0.0);
    return limeLightY;
}

// Limelight Area value read into object
public static double getLimeLightArea() {
    double limeLightArea = ta.getDouble(0.0);
    return limeLightArea;
}

// Limelight Valid Target read into object
public static double getLimeLightV() {
    double limeLightV = tv.getDouble(0.0);
    return limeLightV;
}

// Limelight Skew read into object
public static double getLimeLightS() {
    double limeLightS = ts.getDouble(0.0);
    return limeLightS;
}

// Limelight Shortest Sidelength read into object
public static double getLimeLighttshort() {
    double limeLighttshort = tshort.getDouble(0.0);
    return limeLighttshort;
}

// Limelight Longest Sidelength read into object
public static double getLimeLighttlong() {
    double limeLighttlong = tlong.getDouble(0.0);
    return limeLighttlong;
}

// Limelight Horizontal Sidelength read into object
public static double getLimeLightthor() {
    double limeLightthor = thor.getDouble(0.0);
    return limeLightthor;
}

// Limelight Vertical Sidelength read into object
public static double getLimeLighttvert() {
    double limeLighttvert = tvert.getDouble(0.0);
    return limeLighttvert; 
}
}
