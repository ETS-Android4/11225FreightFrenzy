package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;

@TeleOp(name="TeleOp test peripherals")
//@Disabled
public class TeleopPeripheralTest extends LinearOpMode {

    Hardware22 robot;

    @Override
    public void runOpMode() {
        // Will have:
        // Tower motor
        // Lift motor
        // Collector motor
        // Dump servo
        robot = new Hardware22(hardwareMap);

        robot.dumpServo.setPosition(Constants.collectPosition);

        waitForStart();
        while (opModeIsActive()) {
            // Update controls
            ControlConfig.update(gamepad1, gamepad2);
            // Dump servo
            if (ControlConfig.dumpServo) {
                robot.dumpServo.setPosition(Constants.dumpPosition);
            } else if (ControlConfig.collectServo) {
                robot.dumpServo.setPosition(Constants.collectPosition);
            }

            // Tower motor
            if (ControlConfig.duckWheelForward) {
                robot.towerMotor.setPower(Constants.towerWheelSpeed);
            } else if (ControlConfig.duckWheelBackward) {
                robot.towerMotor.setPower(-Constants.towerWheelSpeed);
            } else {
                robot.towerMotor.setPower(0);
            }

            // Lift motor
            //TODO make it have set levels
            if (ControlConfig.liftBucket) {
                robot.liftMotor.setPower(1.0);
            } else if (ControlConfig.lowerBucket) {
                robot.dumpServo.setPosition(Constants.collectPosition);
                robot.liftMotor.setPower(-1.0);
            } else {
                robot.liftMotor.setPower(0);
            }

            // Collection motor
            if (ControlConfig.collectWheel) {
                robot.collectionMotor.setPower(1.0);
            } else if (ControlConfig.unCollectWheel) {
                robot.collectionMotor.setPower(-1.0);
            } else {
                robot.collectionMotor.setPower(0);
            }

        }
    }
}