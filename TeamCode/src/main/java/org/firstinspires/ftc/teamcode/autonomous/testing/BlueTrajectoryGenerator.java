package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.PathType;
import org.firstinspires.ftc.teamcode.autonomous.enums.ParkingMethod;
import org.firstinspires.ftc.teamcode.autonomous.enums.Position;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

import kotlin.NotImplementedError;

public class BlueTrajectoryGenerator extends TrajectoryGenerator {
    Position position;
    ParkingMethod parkingMethod;

    public BlueTrajectoryGenerator(SampleMecanumDrive drive, Position position, ParkingMethod parkingMethod) {
        super(drive);
        this.position = position;
        this.parkingMethod = parkingMethod;
    }

    public ArrayList<ArrayList<Trajectory>> generateTrajectories() {
        ArrayList<Double[]> trajectory1 = new ArrayList<>();
        ArrayList<Double[]> trajectory2 = new ArrayList<>();
        ArrayList<Double[]> trajectory3 = new ArrayList<>();

        ArrayList<ArrayList<Trajectory>> finalTrajs = new ArrayList<>();

        if (position == Position.BACK) {
            Vector2d vector = new Vector2d(8.0, 61.5);
            Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            // moving to the shipping hub
            generateTrajectoryListItem(-7, 45, 255, PathType.LINE_TO_LINEAR, trajectory1);
            if (parkingMethod == ParkingMethod.WALL) {
                // Forward movement to avoid carossel
                generateTrajectoryListItem(-50, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // getting in position to cross the field and park
               generateTrajectoryListItem(-50, 68, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // hugging the wall and moving into the warehouse
               generateTrajectoryListItem(50, 68.3, 0, PathType.LINE_TO_LINEAR, trajectory3);

            } else if (parkingMethod == ParkingMethod.BARRIER) {
                // getting in position to cross the field and park
                generateTrajectoryListItem(7, 50, 180, PathType.LINE_TO_LINEAR, trajectory3);
                                                                         generateTrajectoryListItem(50, 40, 180, PathType.LINE_TO_LINEAR, trajectory3);
            }

            ArrayList<Trajectory> compTraj1 = compileTrajectoryList(startPose, trajectory1);
            ArrayList<Trajectory> compTraj2 = new ArrayList<>();
            ArrayList<Trajectory> compTraj3 = compileTrajectoryList(compTraj1.get(compTraj1.size() - 1).end(), trajectory3);


            finalTrajs.add(compTraj1);
            finalTrajs.add(compTraj2);
            finalTrajs.add(compTraj3);


        } else if (position == Position.FRONT) {
            Vector2d vector = new Vector2d(-32.25, 61.5);
            Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

            drive.setPoseEstimate(startPose);

            // moving to the shipping hub
            generateTrajectoryListItem(-57, 38, 270, 0, PathType.LINE_TO_LINEAR, trajectory1);
            generateTrajectoryListItem(-30, 24, 90, 0, PathType.SPLINE_TO_LINEAR, trajectory1);
            // getting in position to dump
            generateTrajectoryListItem(-12, 36.5, PathType.LINE_TO_CONSTANT, trajectory1);

            // back up to avoid duck/tse
            generateTrajectoryListItem(-17, 50, PathType.LINE_TO_CONSTANT, trajectory2);

            // Approaching to duck wheel
            generateTrajectoryListItem(-60, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory2);
            // At duck wheel
            generateTrajectoryListItem(-60, 56.25 + 2.2, 0, PathType.LINE_TO_LINEAR, trajectory2);

            if (parkingMethod == ParkingMethod.WALL) {
                // Forward movement to avoid carossel
                generateTrajectoryListItem(-40, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // getting in position to cross the field and park
                generateTrajectoryListItem(-40, 68, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // hugging the wall and moving into the warehouse
                generateTrajectoryListItem(50, 72, 0, PathType.LINE_TO_LINEAR, trajectory3);
            } else if (parkingMethod == ParkingMethod.BARRIER) {
                // Forward movement to avoid carossel
                generateTrajectoryListItem(-50, 48, 0, PathType.LINE_TO_LINEAR, trajectory3);
                // Forward move to warehouse over barrier
                generateTrajectoryListItem(50, 48, 0, PathType.LINE_TO_LINEAR, trajectory3);
            }

            ArrayList<Trajectory> compTraj1 = compileTrajectoryList(startPose, trajectory1);
            ArrayList<Trajectory> compTraj2 = compileTrajectoryList(compTraj1.get(compTraj1.size() - 1).end(), trajectory2);
            ArrayList<Trajectory> compTraj3 = compileTrajectoryList(compTraj2.get(compTraj2.size() - 1).end(), trajectory3);

            finalTrajs.add(compTraj1);
            finalTrajs.add(compTraj2);
            finalTrajs.add(compTraj3);
        }
        return finalTrajs;
    }
}
