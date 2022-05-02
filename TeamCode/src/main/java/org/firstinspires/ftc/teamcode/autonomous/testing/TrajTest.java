package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.competition.util.Hardware22;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TrajTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        Hardware22 robot = new Hardware22(hardwareMap);
        SampleMecanumDrive drive = robot.drive;

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets current pos of robot
        Vector2d vector = new Vector2d(-11.6, 42);
        Pose2d startPose = new Pose2d(vector, Math.toRadians(-270));
        drive.setPoseEstimate(startPose); // This line is important

        waitForStart();
//
        /*-32.25, 61.5, Math.toRadians(0)*/
        TrajectorySequence traj = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(14, 63.5, Math.toRadians(0)), Math.toRadians(0))
                .lineTo(new Vector2d(54, 63.5))
                .lineTo(new Vector2d(14, 63.5))
                .splineToSplineHeading(startPose, Math.toRadians(270))
                .addTemporalMarker(2, () -> {
                    robot.liftMotor.setPower(0.5);
                })
                .addTemporalMarker(3, () -> {
                    robot.liftMotor.setPower(0);
                })
//                                    .splineToSplineHeading(new Pose2d(57.5, 63.5, Math.toRadians(0)), 0)

//                                    .splineToLinearHeading(new Pose2d(22.7, 62.9, Math.toRadians(0)), Math.toRadians(-180))
//                                    .splineTo(new Vector2d(32, -21), 0)
//                                    .splineTo(new Vector2d(35, -10), Math.toRadians(270))
                .build();

        drive.followTrajectorySequence(traj);
    }
}
