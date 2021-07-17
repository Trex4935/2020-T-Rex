package frc.robot;

import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class TrajectoryRepo {

        // General Trajectory
        public static final String trajectoryJSONSim = Constants.pathSim + "Slalom.wpilib.json";
        public static final String trajectoryJSONRobot = Constants.pathRobot + "Back5_0.wpilib.json";
        public static final String trajectoryJSONRobotAlt = Constants.pathRobot + "Back5_0.wpilib.json";
        public static final String trajectoryJSONRobotDefault = Constants.pathRobot + "Back5_0.wpilib.json";
        public static final Trajectory trajectory = TrajectoryContainer.makeTrajectory(trajectoryJSONRobot);
        public static final Trajectory trajectoryAlt = TrajectoryContainer.makeTrajectory(trajectoryJSONRobot);
        public static final Trajectory trajectoryDefault = TrajectoryContainer.makeTrajectory(trajectoryJSONRobot);


        // Autonomous Trajectory
        // Build Trajectory objects to be consumed by auto commands

        public static final Trajectory trajectoryFront = TrajectoryContainer
                        .makeTrajectory(Constants.pathRobot + "Back5_0.wpilib.json");

        public static final Trajectory trajectoryBack = TrajectoryContainer
                        .makeTrajectory(Constants.pathRobot + "Front5_0.wpilib.json");
}
