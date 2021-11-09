package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.autonomous.PathType;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class TrajectoryGenerator {
    SampleMecanumDrive drive;

    public TrajectoryGenerator(SampleMecanumDrive drive) {
        this.drive = drive;
    }

    /**
     * @param x            - The x amount the trajectory should go
     * @param y            - The y amount the trajectory should go
     * @param heading      - The heading in degrees that the robot should end up at (if applicable)
     * @param finalHeading
     * @param pathType     - The type of path used
     * @return - An integer array that can be appended to a trajectory path and executed
     */
    public Double[] generateTrajectoryListItem(double x, double y, double heading, double finalHeading, PathType pathType) {
        if (pathType != PathType.SPLINE_TO_LINEAR) {
            throw new IllegalArgumentException("For this generator, path type must be splineToLinear");
        }

        double pathDouble = pathType.getValue();
        return new Double[]{x, y, heading, finalHeading, pathDouble};
    }

    public Double[] generateTrajectoryListItem(double x, double y, double heading, PathType pathType) {
        if (pathType != PathType.LINE_TO_LINEAR) {
            throw new IllegalArgumentException("For this generator, path type must be lineToLinear");
        }

        double pathDouble = pathType.getValue();
        return new Double[]{x, y, heading, 0.0, pathDouble};
    }

    public Double[] generateTrajectoryListItem(double x, double y, PathType pathType) {
        if (pathType != PathType.LINE_TO_CONSTANT) {
            throw new IllegalArgumentException("For this generator, path type must be lineToConstant");
        }

        double pathDouble = pathType.getValue();
        return new Double[]{x, y, 0.0, 0.0, pathDouble};
    }

    // Execute a list of trajectories as generated by the generateTrajectoryListItem methods
    public Pose2d executeTrajectoryList(Pose2d start, ArrayList<Trajectory> arr) {
        for (Trajectory item : arr) {
            drive.followTrajectory(item);
        }
        return arr.get(arr.size() - 1).end();
    }

    // TODO write this method
    public ArrayList<Trajectory> compileTrajectoryList(Pose2d start, ArrayList<Double[]> arr) {
        ArrayList<Trajectory> compiled = new ArrayList<>();
        Pose2d previous = start;
        for (Double[] item : arr) {
            PathType type = PathType.valueOf(item[4].intValue());
            Trajectory traj;
            switch (type) {
                case LINE_TO_CONSTANT:
                    traj = drive.trajectoryBuilder(previous, true)
                            .lineToConstantHeading(new Vector2d(item[0], item[1]))
                            .build();
                    break;
                case LINE_TO_LINEAR:
                    traj = drive.trajectoryBuilder(previous, true)
                            .lineToLinearHeading(new Pose2d(item[0], item[1], Math.toRadians(item[2])))
                            .build();
                    break;
                case SPLINE_TO_LINEAR:
                    traj = drive.trajectoryBuilder(previous, true)
                            .splineToLinearHeading(new Pose2d(item[0], item[1], Math.toRadians(item[2])), Math.toRadians(item[3]))
                            .build();
                    break;
                default:
                    throw new IllegalStateException("Unexpected value: " + type);
            }
            compiled.add(traj);
            previous = traj.end();
        }
        return compiled;
    }
}
