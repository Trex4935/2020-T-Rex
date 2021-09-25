// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import frc.robot.Extensions.DriveEncoders;
import frc.robot.Extensions.Limelight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // Initialize the tabs on the Shuffleboard
  //public Init_Dashboard dashboard;

  // Limelight object
  public Limelight limelight;

  // Gyro object
  //public AHRS gyro;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Initializing the Shuffleboard Dashboard to that values can be pulled for
    // other systems
    //dashboard = new Init_Dashboard();

    // Initialize the lime light
    limelight = new Limelight();

    // Initalize the gyro
    //gyro = new AHRS(SPI.Port.kMXP);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    /// Outputting running robot data to SmartDashboard widgets

    // Drive Left Axis Widget
    //dashboard.driveLeftAxisWidget.setNumber(RobotContainer.controller.getRawAxis(Constants.leftTankAxis));
    SmartDashboard.putNumber("Drive Left-Axis", RobotContainer.controller.getRawAxis(Constants.leftTankAxis));
    // Drive Right Axis Widget
    //dashboard.driveRightAxisWidget.setNumber(RobotContainer.controller.getRawAxis(Constants.rightTankAxis));
    SmartDashboard.putNumber("Drive Right-Axis", RobotContainer.controller.getRawAxis(Constants.rightTankAxis));
    // Shooter Current running RPM Widget
    //dashboard.shooterCurrentRpm.setNumber(Shooter.currentRpm);
    SmartDashboard.putNumber("Shooter Current RPM", Shooter.currentRpm);
    // Magazine Fully Loaded Status
    //dashboard.sensorShooter.setBoolean(Magazine.getShooterSensor()); 
    SmartDashboard.putBoolean("Mag Fully Loaded", Magazine.getShooterSensor());
    
    // Update Encoder values
    //dashboard.leftFrontEncoder.setNumber(DriveEncoders.lfEncoderValue);
    //dashboard.leftRearEncoder.setNumber(DriveEncoders.lrEncoderValue);
    //dashboard.rightFrontEncoder.setNumber(DriveEncoders.rfEncoderValue);
    //dashboard.rightRearEncoder.setNumber(DriveEncoders.rrEncoderValue);


    // Gyro current heading Widget
    //dashboard.gyroCurrentPosition.setNumber(DriveTrain.ahrs.getAngle());
    SmartDashboard.putNumber("Gyro Position", DriveTrain.ahrs.getAngle());
    
    // Limelight X Axis Widget
    //dashboard.limeLightX.setNumber(Limelight.getLimeLightX());
    SmartDashboard.putNumber("limeLight X", Limelight.getLimeLightX());
    // Limelight Y Axis Widget
    //dashboard.limeLightY.setNumber(Limelight.getLimeLightY());
    SmartDashboard.putNumber("limeLight Y", Limelight.getLimeLightY());
    // Limelight Area Widget
    //dashboard.limeLightArea.setNumber(Limelight.getLimeLightArea());
    SmartDashboard.putNumber("limeLight Area", Limelight.getLimeLightArea());
    // Limelight Valid Target Widget
    //dashboard.limeLightV.setNumber(Limelight.getLimeLightV());
    SmartDashboard.putNumber("limeLight Valid Target", Limelight.getLimeLightV());
    // Limelight Skew Widget
    //dashboard.limeLightS.setNumber(Limelight.getLimeLightS());
    SmartDashboard.putNumber("limeLight Skew", Limelight.getLimeLightS());
    // Limelight Shortest Sidelength Widget
    //dashboard.limeLightS.setNumber(Limelight.getLimeLighttshort());
    SmartDashboard.putNumber("limeLight Short Sidelength", Limelight.getLimeLighttshort());
    // Limelight Longest Sidelength Widget
    //dashboard.limeLighttlong.setNumber(Limelight.getLimeLighttlong());
    SmartDashboard.putNumber("limeLight Long Sidelength", Limelight.getLimeLighttlong());
    // Limelight Horizontal Sidelength Widget
    //dashboard.limeLightthor.setNumber(Limelight.getLimeLightthor());
    SmartDashboard.putNumber("limelight Horizontal Sidelength", Limelight.getLimeLightthor());
    // Limelight Vertical Sidelength Widget
    //dashboard.limeLighttvert.setNumber(Limelight.getLimeLighttvert());
    SmartDashboard.putNumber("limeLight Vertical Sidelength", Limelight.getLimeLighttvert());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.driveTrain.stopDriveTrain();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    //RobotContainer.station = DriverStation.getInstance().getLocation();
    // if (RobotContainer.station == 1){
    //   m_robotContainer.driveTrain.resetOdometry(TrajectoryRepo.trajectory.getInitialPose());
    // }
    // else if(RobotContainer.station == 2){
    //   m_robotContainer.driveTrain.resetOdometry(TrajectoryRepo.trajectoryAlt.getInitialPose());      
    // }
    // else{
    //   m_robotContainer.driveTrain.resetOdometry(TrajectoryRepo.trajectoryDefault.getInitialPose());      
    // }
    //m_robotContainer.driveTrain.resetOdometry(TrajectoryRepo.trajectory.getInitialPose());
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //m_robotContainer.driveTrain.initOdometry()
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // System.out.println(dashboard.maxSpeed.getDouble(Constants.speedLimitDefault));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
