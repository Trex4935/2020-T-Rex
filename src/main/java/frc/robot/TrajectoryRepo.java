package frc.robot;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class TrajectoryRepo {

    // General Trajectory
    public static final String trajectoryJSONSim = Constants.pathSim + "Slalom.wpilib.json";
    public static final String trajectoryJSONRobot = Constants.pathRobot + "Back5_0.wpilib.json";
    public static final Trajectory trajectory = TrajectoryContainer.makeTrajectory(trajectoryJSONRobot);

    // Autonomous Trajectory

    // Shooter Summer 
    public static final Trajectory trajectoryFront = TrajectoryContainer
    .makeTrajectory(Constants.pathRobot + "Back5_0.wpilib.json");  

    
    public static final Trajectory trajectoryBack = TrajectoryContainer
    .makeTrajectory(Constants.pathRobot + "Front5_0.wpilib.json");  
}
