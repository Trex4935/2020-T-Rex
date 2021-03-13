// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Extensions;

/** Add your docs here. */
public class DriveEncoders {

    public static double lfEncoderValue;
    public static double lrEncoderValue;
    public static double rfEncoderValue;
    public static double rrEncoderValue; 

    public DriveEncoders(){
        lfEncoderValue = 0;
        lrEncoderValue = 0;
        rfEncoderValue = 0;
        rrEncoderValue = 0;
    }

    public void SetDriveEncoders(double leftFront,double leftRear,double rightFront,double rightRear){
        lfEncoderValue = leftFront;
        lrEncoderValue = leftRear;
        rfEncoderValue = rightFront;
        rrEncoderValue = rightRear;
    }

}
