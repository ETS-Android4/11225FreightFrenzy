package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;
import org.firstinspires.ftc.teamcode.autonomous.PathType;
import org.firstinspires.ftc.teamcode.autonomous.enums.Color;
import org.firstinspires.ftc.teamcode.autonomous.enums.Position;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

/*
 * This is an example of a more complex path to really test the tuning.
 */
//@Disabled
@Autonomous(group = "drive")
public class AutoInputTest extends LinearOpMode {
    Hardware22 robot;
    SampleMecanumDrive drive;
    TrajectoryGenerator generator;

    Color color;
    Position position;
    ParkingMethod parkingMethod;
    long delay = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware22(hardwareMap);
        drive = robot.drive;
        generator = robot.generator;

        // Color

        telemetry.addLine("Color? x for blue, b for red");
        telemetry.update();

        while (true){
            if (gamepad2.x) {
                color = Color.BLUE;
                break;
            } else if (gamepad2.b){
                color = Color.RED;
                break;
            }
        }
        telemetry.addLine("Color confirmed, " + color);

        // Position

        telemetry.addLine("Position? front or back, dpad up v down");
        telemetry.update();

        while (true) {
            if (gamepad2.dpad_up) {
                position = Position.FRONT;
                break;
            } else if (gamepad2.dpad_down) {
                position = Position.BACK;
                break;
            }
        }

        telemetry.addLine("Position confirmed, " + position);
        telemetry.update();
        sleep(500);

        // Parking method

        telemetry.addLine("Parking method? wall or barrier, dpad right v left");
        telemetry.update();

        while (true) {
            if (gamepad2.dpad_left) {
                parkingMethod = ParkingMethod.BARRIER;
                break;
            } else if (gamepad2.dpad_right) {
                parkingMethod = ParkingMethod.WALL;
                break;
            }
        }

        telemetry.addLine("Parking method confirmed, " + parkingMethod);
        telemetry.update();
        sleep(500);

        // Delay

        boolean buttonUnpressed = true;
        while (true) {
            telemetry.addData("Delay? RB to add 10 ms, LB to add 100ms, y done", delay);
            telemetry.update();
            if (gamepad2.right_bumper && buttonUnpressed) {
                delay += 10;
                buttonUnpressed = false;
            } else if (gamepad2.left_bumper && buttonUnpressed) {
                delay += 100;
                buttonUnpressed = false;
            } else if (gamepad2.y) {
                break;
            } else if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                buttonUnpressed = true;
            }
        }
        telemetry.addLine("Delay confirmed, " + delay);
        telemetry.update();

        ArrayList<ArrayList<Trajectory>> trajs;
        TrajectoryGenerator gen;
        if (color == Color.RED) {
            gen = new RedTrajectoryGenerator(drive, position, parkingMethod);
            trajs = ((RedTrajectoryGenerator) gen).generateTrajectories();
        } else {
            gen = new BlueTrajectoryGenerator(drive, position, parkingMethod);
            trajs = ((BlueTrajectoryGenerator) gen).generateTrajectories();
        }


        waitForStart();

        telemetry.addData("Color", color);
        telemetry.addData("Position", position);
        telemetry.addData("Delay", delay);
        telemetry.update();
        sleep(delay);

        // TODO execute detection
        telemetry.addLine("Traj 1");
        telemetry.update();
        gen.executeTrajectoryList(trajs.get(0)); // going to shipping hub
        telemetry.addLine("Traj 2");
        telemetry.update();
        // TODO dump at correct height
        if (position == Position.FRONT) {
            gen.executeTrajectoryList(trajs.get(1)); // going to duck wheel
            sleep(2000);
            // TODO deliver duck
        }

        telemetry.addLine("Traj 3");
        telemetry.update();
        gen.executeTrajectoryList(trajs.get(2)); // going to park in warehouse
    }
}