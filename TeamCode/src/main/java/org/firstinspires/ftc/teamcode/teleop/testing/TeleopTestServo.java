package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Hardware22;

import java.util.Locale;

@TeleOp(name="TeleOp test servo")
//@Disabled
public class TeleopTestServo extends LinearOpMode {

    //Hardware22 robot = new Hardware22();

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

    double frontLeft;
    double rearLeft;
    double frontRight;
    double rearRight;

    double forward;
    double right;
    double clockwise;

    double powerMultiplier = 1;
    double deadZone = Math.abs(0.2);

    double temp;
    double side;

    double currentAngle;

    Boolean buttonPress=false;
    Boolean servoSwitch=false;

    Boolean buttonPress2=false;
    Boolean servoSwitch2=false;

    Boolean buttonPress3 = false;
    Boolean servoSwitch3 = false;

    Boolean aButtonPress = false;
    Boolean launcherOn = false;

    Boolean xButtonPress = false;
    Boolean servoPosition = false;

    @Override
    public void runOpMode() {
        Hardware22 robot = null;
        try {
            robot = new Hardware22(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine(e.getMessage());
            telemetry.addData("Line num", e.getStackTrace()[0].getLineNumber());
            telemetry.update();
            sleep(10000);
        }

        robot.tseArmServo.setPosition(Constants.tseArmInitPosition);
        robot.tseRodServo.setPosition(Constants.tseRodInitPosition);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ControlConfig.update(gamepad1, gamepad2);

            if (ControlConfig.toggleTseArmServo) {
                robot.tseArmServo.setPosition(Constants.tseArmActivePosition);
            }
            if (ControlConfig.toggleTseRodServo) {
                robot.tseRodServo.setPosition(Constants.tseRodActivePosition);
            }
        }
    }


    //----------------------------------------------------------------------------------------------
    // DO NOT WRITE CODE BELOW THIS LINE
    //----------------------------------------------------------------------------------------------
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

        // telemetry.addData("currentAngle", "%.1f", currentAngle);
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}