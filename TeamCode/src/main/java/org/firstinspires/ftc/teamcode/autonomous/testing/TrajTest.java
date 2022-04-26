package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.util.Hardware22;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class TrajTest extends LinearOpMode {
   @Override
   public void runOpMode() throws InterruptedException {
      FtcDashboard dashboard = FtcDashboard.getInstance();
      telemetry = dashboard.getTelemetry();

      Hardware22 robot = new Hardware22(hardwareMap);
      SampleMecanumDrive drive = robot.drive;

      // Sets current pos of robot
      drive.setPoseEstimate(new Pose2d(-32.25, 61.5, Math.toRadians(270))); // This line is important

      waitForStart();
//
//      Vector2d vector = new Vector2d(-32.25, 61.5);
//      Pose2d startPose = new Pose2d(vector, Math.toRadians(270));
      /*-32.25, 61.5, Math.toRadians(0)*/
      Trajectory traj = drive.trajectoryBuilder(new Pose2d(-32.25, 61.5, Math.toRadians(270)), false)
              // TODO try to make it so that it only splines corners
              .splineTo(new Vector2d(-56, 37), Math.toRadians(270))
              .splineTo(new Vector2d(-23, 25), 0)
              .build();
      drive.followTrajectory(traj);
   }
}
