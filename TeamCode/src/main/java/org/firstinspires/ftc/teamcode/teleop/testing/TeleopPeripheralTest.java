package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;

import java.util.Locale;

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