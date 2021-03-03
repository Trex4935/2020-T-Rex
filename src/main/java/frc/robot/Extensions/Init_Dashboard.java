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
                        shooterCurrentRpm, limelightOnOff, gyroCurrentPosition,
                        sensorMagazine, sensorIntake, sensorShooter,
                        limeLightX, limeLightY, limeLightArea;

        // Constructor to build all Shuffleboard widgets
        public Init_Dashboard() {

                // Object creation for the Input widgets

                // Create the Max Speed input widget used to set the max speed of the robot
                maxSpeed = inputsTab.add("Max Speed", Constants.speedLimitDefault)
                                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
                                .withPosition(0, 1).getEntry();

                // Create the Drive Type widget to set the drive between Arcade and Tank with a boolean button
                driveType = inputsTab.add("Drive Type = Tank", Constants.arcadeDrive)
                                .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 0).withSize(2, 1).getEntry();

                // Create the Shooter RPM widget to be able to set the max RPM of the shooter
                shooterTargetRpm = inputsTab.add("Shooter Target RPM", Constants.targetRPM)
                                .withWidget(BuiltInWidgets.kTextView).withPosition(0, 2).withSize(2, 1).getEntry();

                // Create the Limelight Blinker Widget to be able to turn the light on and off
                //limelightOnOff = inputsTab.add("Limelight On_Blink_Off", 3)
                //                .withWidget(BuiltInWidgets.kComboBoxChooser).withProperties(Map.of("off", 1, "blink", 2, "on", 3))
                //                .withPosition(2, 0)
                //                .withSize(3, 3)
                //                .getEntry();

                // Object creation for the driver data dispaly widgets

                // Create the Drive Left Axis widget to display the position of the left drive input
                driveLeftAxisWidget = driverInfoTab.add("Drive Left-Axis", 0.0).withWidget("Text View")
                                .withPosition(0, 0).withSize(1, 1).getEntry();

                // Create the Drive Right Axis widget to display the position of the right drive input
                driveRightAxisWidget = driverInfoTab.add("Drive Right-Axis", 1.0).withWidget("Text View")
                                .withPosition(1, 0).withSize(1, 1).getEntry();

                // Create the Shooter Current RPM widget to display the current running RPM of the shooter, 
                //      as read from the motor encoder.
                shooterCurrentRpm = driverInfoTab.add("Shooter Current RPM", 0.00).withWidget("Text View")
                                .withPosition(3, 0).withSize(2, 1).getEntry();

                // Create the Gyro widget to display the current Gyro header reading.
                gyroCurrentPosition = driverInfoTab.add("Current Gyro Heading", 0).withWidget("Gyro")
                                .withPosition(6, 0).withSize(2, 2).getEntry();
        
                // Object Creation for Magazine Status widgets

                // Create the Intake status widget to check operation of the input roller Smackna
                sensorIntake = magazineStatusTab.add("Intake Status", false).withWidget("Boolean Box")
                                .withPosition(0, 0).withSize(1, 1).getEntry();

                // Create the Magazine status widget to check operation of the magazine input Smackna
                sensorMagazine = magazineStatusTab.add("Magazine Status", false).withWidget("Boolean Box")
                                .withPosition(0, 1).withSize(1, 1).getEntry();

                // Create the Shooter status widget to check operation of the shooter Smackna
                sensorShooter = magazineStatusTab.add("Shooter Status", false).withWidget("Boolean Box")
                                .withPosition(0, 2).withSize(1, 1).getEntry();

                // Object Creation for Limelight Status widgets

                // Create Limelight status X Axis Widget
                limeLightX = limelightInfoTab.add("Limelight X Value", 0.0).withWidget("Text View")
                                .withPosition(0, 0).withSize(1, 1).getEntry();

                // Create Limelight status Y Axis Widget
                limeLightY = limelightInfoTab.add("Limelight Y Value", 0.0).withWidget("Text View")
                                .withPosition(1, 0).withSize(1, 1).getEntry();

                // Create Limelight status Area Widget
                limeLightArea = limelightInfoTab.add("Limelight Area Value", 0.0).withWidget("Text View")
                                .withPosition(2, 0).withSize(2, 1).getEntry();

                }
}