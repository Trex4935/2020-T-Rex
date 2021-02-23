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

        private ShuffleboardTab inputsTab = Shuffleboard.getTab("Inputs");
        private ShuffleboardTab driverInfoTab = Shuffleboard.getTab("Driver Info");
        public NetworkTableEntry maxSpeed, driveType, driveLeftAxisWidget, driveRightAxisWidget, shooterTargetRpm,
                        shooterCurrentRpm, gyroCurrentPosition;

        // Constructor to build all Shuffleboard widgets
        public Init_Dashboard() {

                // Object creation for the Input widgets
                maxSpeed = inputsTab.add("Max Speed", Constants.speedLimitDefault)
                                .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1))
                                .withPosition(0, 1).getEntry();

                driveType = inputsTab.add("Drive Type = Tank", Constants.arcadeDrive)
                                .withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 0).withSize(2, 1).getEntry();

                shooterTargetRpm = inputsTab.add("Shooter Target RPM", Constants.targetRPM)
                                .withWidget(BuiltInWidgets.kTextView).withPosition(0, 2).withSize(2, 1).getEntry();

                // Object creation for the data dispaly widgets
                driveLeftAxisWidget = driverInfoTab.add("Drive Left Axis", 1).withWidget(BuiltInWidgets.kTextView)
                                .withPosition(0, 0).withSize(1, 1).getEntry();

                driveRightAxisWidget = driverInfoTab.add("Drive Right Axis", 2).withWidget(BuiltInWidgets.kTextView)
                                .withPosition(1, 0).withSize(1, 1).getEntry();

                shooterCurrentRpm = driverInfoTab.add("Shooter Current RPM", 3).withWidget(BuiltInWidgets.kTextView)
                                .withPosition(3, 0).withSize(2, 1).getEntry();

                gyroCurrentPosition = driverInfoTab.add("Current Gyro Heading", 4).withWidget(BuiltInWidgets.kGyro)
                                .withPosition(6, 0).withSize(2, 2).getEntry();
        }
}