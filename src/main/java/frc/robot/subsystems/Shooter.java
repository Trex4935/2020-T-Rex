// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  TalonFX shooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {
    // initilize the motor
    shooterMotor = new TalonFX(Constants.shooterMotorID);
    shooterMotor.setInverted(false);
    shooterMotor.configFactoryDefault();
    shooterMotor.configNeutralDeadband(0.001);

    // setting up the pid
    shooterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    // set motor limits
    // normal output forward and reverse = 0% ... i.e. stopped
    shooterMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    shooterMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);

    // Max output forward and reverse = 100%
    shooterMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    // put in the values for the PIDF ... they are coming
    shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
    shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
    shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
    shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Shoot the ball at a certain speed
  // Press button to shoot ball

  public void shoot() {
    shooterMotor.set(ControlMode.PercentOutput, Constants.shooterSpeed);
  }

  public void shootStop() {
    shooterMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

}
/*
 * 
 * /* Hardware *\ TalonFX _talon = new TalonFX(1); Joystick _joy = new
 * Joystick(0);
 * 
 * /* String for output *\ StringBuilder _sb = new StringBuilder(); - DONT NEED
 * 
 * /* Loop tracker for prints *\ int _loops = 0; - DONT NEED
 * 
 * public void robotInit() { /* Factory Default all hardware to prevent
 * unexpected behaviour *\ _talon.configFactoryDefault();
 * 
 * /* Config neutral deadband to be the smallest possible *\
 * _talon.configNeutralDeadband(0.001);
 * 
 * /* Config sensor used for Primary PID [Velocity] *\
 * _talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
 * Constants.kPIDLoopIdx, Constants.kTimeoutMs);
 * 
 * 
 * /* Config the peak and nominal outputs *\
 * _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
 * _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
 * _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
 * _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);
 * ---------------------------------------------- /* Config the Velocity closed
 * loop gains in slot0 *\ _talon.config_kF(Constants.kPIDLoopIdx,
 * Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
 * _talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP,
 * Constants.kTimeoutMs); _talon.config_kI(Constants.kPIDLoopIdx,
 * Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
 * _talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD,
 * Constants.kTimeoutMs); /* Talon FX does not need sensor phase set for its
 * integrated sensor This is because it will always be correct if the selected
 * feedback device is integrated sensor (default value) and the user calls
 * getSelectedSensor* to get the sensor's position/velocity.
 * 
 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
 * sensor-phase \ // _talon.setSensorPhase(true);
 */