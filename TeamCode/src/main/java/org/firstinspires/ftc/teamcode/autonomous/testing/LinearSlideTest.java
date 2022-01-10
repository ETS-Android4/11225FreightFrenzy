package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;

@TeleOp
public class LinearSlideTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Hardware22 robot = new Hardware22(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.dumpServo.setPosition(Constants.collectPosition);

        waitForStart();
        while (true) {
            if (gamepad1.dpad_up) {
                robot.liftMotor.setPower(.5);
            } else if (gamepad1.dpad_down) {
                robot.liftMotor.setPower(-.5);
            } else {
                robot.liftMotor.setPower(0);
            }

            if (gamepad1.a) {
                robot.dumpServo.setPosition(Constants.collectPosition);
            } else if (gamepad1.y) {
                robot.dumpServo.setPosition(Constants.dumpPosition);
            }

            int current = robot.liftMotor.getCurrentPosition();
            telemetry.addLine("got here");
            telemetry.addData("Current pos", current);
            telemetry.update();
        }
    }
}