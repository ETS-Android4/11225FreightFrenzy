package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Hardware22;
import org.firstinspires.ftc.teamcode.autonomous.PathType;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.nio.file.Path;
import java.sql.Array;
import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(group = "drive")
public class AutoBlueTest extends LinearOpMode {

    public static double DISTANCE = 20; //in
    // FIXME make this much MUCH better
    Hardware22 robot;
    SampleMecanumDrive drive;
    TrajectoryGenerator generator;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware22(hardwareMap);
        drive = robot.drive;
        generator = robot.generator;

        Vector2d vector = new Vector2d(-40.75, 61.5);
        Pose2d startPose = new Pose2d(vector, Math.toRadians(270));

        drive.setPoseEstimate(startPose);
        ArrayList<Double[]> trajectory1 = new ArrayList<>();
        generator.generateTrajectoryListItem(-30, 47, 310, 0, PathType.SPLINE_TO_LINEAR, trajectory1);
        generator.generateTrajectoryListItem(-22, 36.5, PathType.LINE_TO_CONSTANT, trajectory1);
        // Approaching to duck wheel
        generator.generateTrajectoryListItem(-60, 56.25, 0, PathType.LINE_TO_LINEAR, trajectory1);
        // At duck wheel
        generator.generateTrajectoryListItem(-60, 56.25 + 1.7, 0, PathType.LINE_TO_LINEAR, trajectory1);


        ArrayList<Double[]> trajectory2 = new ArrayList<>();
        generator.generateTrajectoryListItem(-50, 62.7, 0, PathType.LINE_TO_LINEAR, trajectory2);
        generator.generateTrajectoryListItem(50, 62.7, 0, PathType.LINE_TO_LINEAR, trajectory2);

        ArrayList<Trajectory> compiled1 = generator.compileTrajectoryList(startPose, trajectory1);
        ArrayList<Trajectory> compiled2 = generator.compileTrajectoryList(compiled1.get(compiled1.size() - 1).end(), trajectory2);

        waitForStart();

        if (isStopRequested()) return;

        generator.executeTrajectoryList(compiled1);

        sleep(500);
        robot.towerMotor.setPower(.6);
        sleep(2000);
        robot.towerMotor.setPower(0);
        sleep(500);

        generator.executeTrajectoryList(compiled2);
    }
}