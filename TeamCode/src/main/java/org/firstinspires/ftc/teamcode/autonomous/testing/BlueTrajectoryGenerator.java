package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.PathType;
import org.firstinspires.ftc.teamcode.autonomous.enums.ParkingMethod;
import org.firstinspires.ftc.teamcode.autonomous.enums.Position;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class BlueTrajectoryGenerator extends TrajectoryGenerator {
    Position position;
    ParkingMethod parkingMethod;

    public BlueTrajectoryGenerator(SampleMecanumDrive drive, Position position, ParkingMethod parkingMethod) {
        super(drive);
        this.position = position;
        this.parkingMethod = parkingMethod;
    }

    public ArrayList<ArrayList<Trajectory>> generateTrajectories() {
        ArrayList<ArrayList<Trajectory>> finalTrajs = new ArrayList<>();

        Vector2d vector = new Vector2d(-32.25, 61.5);
        Pose2d startPose = new Pose2d(vector, Math.toRadians(270));


        drive.setPoseEstimate(startPose);

        //duck in the middle in front
        ArrayList<Double[]> trajectory1 = new ArrayList<>();
        generateTrajectoryListItem(-30, 47, 310, 0, PathType.SPLINE_TO_LINEAR, trajectory1);
        generateTrajectoryListItem(-22, 36.5, PathType.LINE_TO_CONSTANT, trajectory1);

        //duck on the right in front
        //TODO paths for duck on the right on the right side

        //duck on the left in front
        //TODO paths for duck on the left on the right side

        ArrayList<Double[]> trajectory2 = new ArrayList<>();
        generateTrajectoryListItem(-50, 66.7, 0, PathType.LINE_TO_LINEAR, trajectory2);
        generateTrajectoryListItem(50, 67.2, 0, PathType.LINE_TO_LINEAR, trajectory2);

        ArrayList<Double[]> trajectory3 = new ArrayList<>();
        // Approaching to duck wheel
        generateTrajectoryListItem(-60, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory3);

        ArrayList<Double[]> trajectory4 = new ArrayList<>();
        // At duck wheel
        generateTrajectoryListItem(-60, 56.25 + 1.7, 0, PathType.LINE_TO_LINEAR, trajectory4);


        ArrayList<Trajectory> compTraj1 = compileTrajectoryList(startPose, trajectory1);
        ArrayList<Trajectory> compTraj2 = compileTrajectoryList(compTraj1.get(compTraj1.size() - 1).end(), trajectory2);
        ArrayList<Trajectory> compTraj3 = compileTrajectoryList(compTraj2.get(compTraj2.size() - 1).end(), trajectory3);
        ArrayList<Trajectory> compTraj4 = compileTrajectoryList(compTraj3.get(compTraj3.size() - 1).end(), trajectory4);

        finalTrajs.add(compTraj1);
        finalTrajs.add(compTraj2);
        finalTrajs.add(compTraj3);
        finalTrajs.add(compTraj4);

        return finalTrajs;
    }
}
