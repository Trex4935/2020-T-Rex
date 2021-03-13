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
import frc.robot.Extensions.Dashboard_Outputs;

public class Shooter extends SubsystemBase {
  TalonFX shooterMotor;
  public static double currentRpm;

  // Allows use of inputs from Shuffleboard
  private Dashboard_Outputs dashOutput;

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

    // Getting Values from Dashboard Output
    dashOutput = new Dashboard_Outputs();

    // set motor limits
    // normal output forward and reverse = 0% ... i.e. stopped
    shooterMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    shooterMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);

    // Max output forward and reverse = 100%
    shooterMotor.configPeakOutputForward(1, Constants.kTimeoutMs);
    shooterMotor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    // put in the values for the PIDF ... they are coming
    shooterMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit_Shooter.kP, Constants.kTimeoutMs);
    shooterMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit_Shooter.kI, Constants.kTimeoutMs);
    shooterMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit_Shooter.kD, Constants.kTimeoutMs);
    shooterMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit_Shooter.kF, Constants.kTimeoutMs);

  }

  @Override
  public void periodic() {

  }

  // Shoot the ball at a certain speed
  // Press button to shoot ball

  public void shoot() {
    shooterMotor.set(ControlMode.PercentOutput, Constants.shooterSpeed);
    currentRpm = (shooterMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx)*600)/2048;

    // _sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
    // _talon.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

  }

  public void shootPID() {
    /**
     * Convert 500 RPM to units / 100ms.
     * 2048 Encoder Units / Revolution
     * 600msu (100ms units) / minute
     * So targetUnits_100ms = (2048 * TargetRPM) / 600msu
     */
    double targetUnits_100ms = (2048 * dashOutput.getShooterTargetRPM()) / 600;
    shooterMotor.set(TalonFXControlMode.Velocity, targetUnits_100ms);
    currentRpm = (shooterMotor.getSelectedSensorVelocity(Constants.kPIDLoopIdx)*600)/2048;
  }

  public void shootStop() {
    shooterMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

}