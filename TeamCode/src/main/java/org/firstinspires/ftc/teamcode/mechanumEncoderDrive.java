package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class mechanumEncoderDrive extends LinearOpMode {

        DcMotor frontLeftMotor;
        DcMotor frontRightMotor;
        DcMotor backLeftMotor;
        DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {

            frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
            frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
            backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
            backRightMotor = hardwareMap.get(DcMotor.class, "back_right");


            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            backRightMotor.setDirection(DcMotor.Direction.REVERSE);



            waitForStart();

        encoderDrive(0.05, 60);

    }

    public void encoderDrive(double speed, double cm) {


}
}
