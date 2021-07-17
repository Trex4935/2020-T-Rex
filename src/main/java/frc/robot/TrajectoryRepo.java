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

    // Take Home 2020
    
    // Galactic search
    public static final Trajectory trajectoryGSearch = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "GSearch.wpilib.json");

    // Auto Nav
    public static final Trajectory trajectorySlalom = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "SlowSlalomReal3.27.wpilib.json");
    public static final Trajectory trajectoryBarrelRacing = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "SLOWBarrelRacingReal27.wpilib.json");
    public static final Trajectory trajectoryBounce = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Bounce27.4.wpilib.json");

    // Shooter Trow
    public static final Trajectory trajectoryShootTrow = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Slalom.wpilib.json");
    public static final Trajectory trajectoryLineForward = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Straight.wpilib.json");
    public static final Trajectory trajectoryLineBackward = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Backward.wpilib.json");

    // Bounce Path
    public static final Trajectory trajectoryBounce1 = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Bounce27.1.wpilib.json");
    public static final Trajectory trajectoryBounce2 = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Bounce27.2.wpilib.json");
    public static final Trajectory trajectoryBounce3 = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Bounce27.3.wpilib.json");
    public static final Trajectory trajectoryBounce4 = TrajectoryContainer
            .makeTrajectory(Constants.pathRobot + "Bounce27.4.wpilib.json");

    // Summer Competition(TBD)
}
