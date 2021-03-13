// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

import java.util.Map;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

/** Add your docs here. */
// Creating a class and methods to be able to load from and write data to the
// Shuffleboard.
public class Init_Dashboard {

        // Create a Shuffleboard tab for Inputs
        private ShuffleboardTab inputsTab = Shuffleboard.getTab("Inputs");
        // Create a Shuffleboard tab for Driver's Info
        private ShuffleboardTab driverInfoTab = Shuffleboard.getTab("Driver Info");
        // Create a Shuffleboard tab for Magazine load status
        private ShuffleboardTab magazineStatusTab = Shuffleboard.getTab("Magazine Status");
        // Create a Shuffleboard tab for Limelight Data
        private ShuffleboardTab limelightInfoTab = Shuffleboard.getTab("Limelight Data");
        // Create the objects used for the widgets in NetworkTables
        public NetworkTableEntry maxSpeed, driveType, driveLeftAxisWidget, driveRightAxisWidget, shooterTargetRpm,
                        shooterCurrentRpm, limelightOnOff, gyroCurrentPosition, sensorMagazine, sensorIntake,
                        sensorShooter, limeLightX, limeLightY, limeLightArea, limeLightV, limeLightS, limeLighttshort,
                        limeLighttlong, limeLightthor, limeLighttvert, leftFrontEncoder, leftRearEncoder,
                        rightFrontEncoder, rightRearEncoder;

        // Constructor to build all Shuffleboard widgets
        public Init_Dashboard() {

                // Call the driver input tab creation
                Init_DriverInfo();

                // Call the limeLight input tab creation
                Init_limeLight(); 

                // Call the magazine input tab creation
                Init_Magazine(); 

                // Call the inputs input tab creation
                Init_Inputs(); 


        }

        // Object creation for the driver data dispaly widgets
        public void Init_DriverInfo() {

                // Left front Encoder
                leftFrontEncoder = driverInfoTab.add("Left Front", 0).withWidget("Text View").withPosition(0, 1)
                                .withSize(1, 1).getEntry();

                // Left Rear Encoder
                leftRearEncoder = driverInfoTab.add("Left Rear", 0).withWidget("Text View").withPosition(0, 2)
                                .withSize(1, 1).getEntry();

                // Right Front Encoder
                rightFrontEncoder = driverInfoTab.add("Right Front", 0).withWidget("Text View").withPosition(1, 1)
                                .withSize(1, 1).getEntry();

                // Right Rear Encoder
                rightRearEncoder = driverInfoTab.add("Right Rear", 0).withWidget("Text View").withPosition(1, 2)
                                .withSize(1, 1).getEntry();

                // Create the Drive Left Axis widget to display the position of the left drive
                // input
                driveLeftAxisWidget = driverInfoTab.add("Drive Left-Axis", 0.0).withWidget("Text View")
                                .withPosition(0, 0).withSize(1, 1).getEntry();

                // Create the Drive Right Axis widget to display the position of the right drive
                driveRightAxisWidget = driverInfoTab.add("Drive Right-Axis", 1.0).withWidget("Text View")
                                .withPosition(1, 0).withSize(1, 1).getEntry();

                // Create the Shooter Current RPM widget to display the current running RPM of
                // the shooter, as read from the motor encoder.
                shooterCurrentRpm = driverInfoTab.add("Shooter Current RPM", 0.00).withWidget("Text View")
                                .withPosition(3, 0).withSize(2, 1).getEntry();

                // Create the Gyro widget to display the current Gyro header reading.
                gyroCurrentPosition = driverInfoTab.add("Current Gyro Heading", 0).withWidget("Gyro").withPosition(6, 0)
                                .withSize(2, 2).getEntry();

        }

        // Object creation for the Limelight data display widgets
        public void Init_limeLight(){ 
 
        // Create Limelight status X Axis Widget
                limeLightX = limelightInfoTab.add("Limelight X Value", 0.0).withWidget("Text View").withPosition(0, 0)
                 .withSize(1, 1).getEntry();

 
        // Create Limelight status Y Axis Widget
                limeLightY = limelightInfoTab.add("Limelight Y Value", 0.0).withWidget("Text View").withPosition(1, 0)
                 .withSize(1, 1).getEntry();

 
        // Create Limelight status Area Widget
                limeLightArea = limelightInfoTab.add("Limelight Area Value", 0.0).withWidget("Text View")
                 .withPosition(2, 0).withSize(2, 1).getEntry();

 
        // Create Limelight status Valid Target Widget
                limeLightV = limelightInfoTab.add("Limelight V Value", 0.0).withWidget("Text View").withPosition(2, 0)
                 .withSize(1, 1).getEntry();

 
        // Create Limelight status Skew Widget
                limeLightS = limelightInfoTab.add("Limelight S Value", 0.0).withWidget("Text View").withPosition(3, 0)
                 .withSize(1, 1).getEntry();

 
        // Create Limelight status Shortest Sidelength Widget
                limeLighttshort = limelightInfoTab.add("Limelight tshort Value", 0.0).withWidget("Text View")
                 .withPosition(4, 0).withSize(1, 1).getEntry();

 
        // Create Limelight status Longest Sidelength Widget
                limeLighttlong = limelightInfoTab.add("Limelight tlong Value", 0.0).withWidget("Text View")
                 .withPosition(5, 0).withSize(1, 1).getEntry();


        // Create Limelight status Horizontal Sidelength Widget
                limeLightthor = limelightInfoTab.add("Limelight thor Value", 0.0).withWidget("Text View")
                 .withPosition(6, 0).withSize(1, 1).getEntry();

 
        // Create Limelight status Vertical Sidelength Widget
                limeLighttvert = limelightInfoTab.add("Limelight tvert Value", 0.0).withWidget("Text View")
                 .withPosition(7, 0).withSize(1, 1).getEntry();
        }

        // Object creation for the Magazine data display widgets
        public void Init_Magazine(){

                sensorIntake = magazineStatusTab.add("Intake Status", false).withWidget("Boolean Box")
                .withPosition(0, 0).withSize(1, 1).getEntry();

        // Create the Magazine status widget to check operation of the magazine input
        // Smackna
                sensorMagazine = magazineStatusTab.add("Magazine Status", false).withWidget("Boolean Box")
                .withPosition(0, 1).withSize(1, 1).getEntry();

        // Create the Shooter status widget to check operation of the shooter Smackna
                sensorShooter = magazineStatusTab.add("Shooter Status", false).withWidget("Boolean Box")
                .withPosition(0, 2).withSize(1, 1).getEntry();
        }

        // Object creation for the inputs data display widgets
        public void Init_Inputs(){

                // Create the Max Speed input widget used to set the max speed of the robot
                maxSpeed = inputsTab.add("Max Speed", Constants.speedLimitDefault)
                                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0.5, "max", 1))
                                .withPosition(0, 1).getEntry();

                // Create the Drive Type widget to set the drive between Arcade and Tank with a
                // boolean button
                driveType = inputsTab.add("Drive Type = Tank", Constants.arcadeDrive)
                                .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 0).withSize(2, 1).getEntry();

                // Create the Shooter RPM widget to be able to set the max RPM of the shooter
                shooterTargetRpm = inputsTab.add("Shooter Target RPM", Constants.targetRPM)
                                .withWidget(BuiltInWidgets.kTextView).withPosition(0, 2).withSize(2, 1).getEntry();
        }
}