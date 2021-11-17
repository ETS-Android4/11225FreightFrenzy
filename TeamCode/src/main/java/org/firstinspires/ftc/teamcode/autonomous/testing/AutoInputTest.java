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

        telemetry.addLine("Position? right or left, dpad left v right");
        telemetry.update();

        while (true) {
            if (gamepad2.dpad_left) {
                position = Position.LEFT;
                break;
            } else if (gamepad2.dpad_right) {
                position = Position.RIGHT;
                break;
            }
        }

        telemetry.addLine("Position confirmed, " + position);
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

        waitForStart();

        telemetry.addData("Color", color);
        telemetry.addData("Position", position);
        telemetry.addData("Delay", delay);
        telemetry.update();
        sleep(10000);

    }
}