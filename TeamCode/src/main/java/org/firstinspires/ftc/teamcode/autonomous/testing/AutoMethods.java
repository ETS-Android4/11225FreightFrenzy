package org.firstinspires.ftc.teamcode.autonomous.testing;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Hardware22;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


/**
 * Created by robotics on 10/12/2017.
 */

public abstract class AutoMethods extends LinearOpMode {

    /* Declare OpMode members. */

    double counts_per_rev = 537.6;
    double counts_per_rev_arm = 1993.6;
    double actual_speed = 0.6;
    double wheelDiameter_cm = 10;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    double widthOfRobot_cm = 40;
    double lengthOfRobot_cm = 33.5;

    double radiusOfRobot_cm = Math.sqrt(((.5*widthOfRobot_cm)*(0.5*widthOfRobot_cm)) + ((0.5*lengthOfRobot_cm)*(0.5*lengthOfRobot_cm)));
    double circumferenceOfRobotCircle_cm = 2 * 3.1415 * radiusOfRobot_cm;


    double counts_per_cm = counts_per_rev * DRIVE_GEAR_REDUCTION / (wheelDiameter_cm * 3.1415);
    double counts_per_degree = DRIVE_GEAR_REDUCTION * counts_per_rev_arm / (360);




    DigitalChannel digitalTouch;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;



    Hardware22 robot;
    private ElapsedTime runtime = new ElapsedTime();

    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    private boolean targetVisible = false;
    String vuforiaCode = "";


    ////METHODS////
    ///Vuforia/////////////////////////////////////////

    ////Vuforia Code////
    public void vuforiaCode(double time) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWAQ1WL/////AAAAGXVdDwxhtUkZvdHHgJOEVsNIDr+6XlP2GxHyPxlCyj6GvdGDrT75jhVpmYXcXxWlLdDukO49wWW1CWWqMcE12j3OVWX9aA8ayXr50unvfcKlIbRDQDNEfCOArmADXfDcoW22JaHvoD4hRhQp6umyV1Av/ceiMWvCETajTt/TebeJMud4EBSm5eyPNKTEabVLoGP9PEUHzC2zD7NziZiQBkQaYa4NpIRgwMzz24E6qnz3mVO4jjLPlwHuzkTBu9/YZvmhdx7dGHCPxl100vjGSlPunKtqJOz679vk0r0T8u/TEdntEIbaQ7rHPXJ57lXaBvOf1aMK4Wk5EEJYTPYkCVi0hxvnTHbkMTKEgGoW/hM8";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        telemetry.update();
        waitForStart();

        targetsSkyStone.activate();

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {

            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    //////CODE WEYRENS ADDED/////
                    if(trackable.getName().equals("Stone Target")){
                        telemetry.addLine("Stone Target Visable!!");
                    }
                    //////////////////////////////////
                    targetVisible = true;

                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            String positionSkystone = ""; //Weyrens Added
            double yPosition = 10000;
            double xPosition = 10000;
            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0), translation.get(1), translation.get(2));
                //////CODE WEYRENS ADDED/////
                yPosition = translation.get(1);

                //values for when camera is 50 cm from stones in landscape mode (camera on left)
                if(yPosition < -150){
                    vuforiaCode = "left";
                    telemetry.addLine("code = left");
                    telemetry.addData("y Position",yPosition);
                    telemetry.update();
                    sleep(1000);
                }
                else if (yPosition > 150){
                    vuforiaCode = "right";
                    telemetry.addLine("code = right");
                    telemetry.addData("y Position",yPosition);
                    telemetry.update();
                    sleep(1000);
                }
                else
                    vuforiaCode = "center";
                    telemetry.addLine("code = center");
                    telemetry.addData("y Position",yPosition);
                    telemetry.update();
                    sleep(1000);

                //////////////////////////////////
                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            }
            else {
                telemetry.addData("Visible Target", "none");
            }

        }

    }

    public void encoderTowerAcc (double speed, double distanceCM,double rampupPercentage, double timeOut) {

        robot.towerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                robot.towerMotor.getCurrentPosition(),
        telemetry.update());

        int NEW_upright_target;

        int FRnewzero = robot.towerMotor.getCurrentPosition();

        if (opModeIsActive()) {

            NEW_upright_target = robot.towerMotor.getCurrentPosition() + (int) (distanceCM * counts_per_cm);

            double FRstartEncoder = robot.frontRight.getCurrentPosition();

            robot.towerMotor.setTargetPosition(NEW_upright_target);

            // Turn On RUN_TO_POSITION
            robot.towerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();


            while (opModeIsActive() &&
                    (runtime.seconds() < timeOut) &&
                    (robot.towerMotor.isBusy())) {
                double distance = Math.abs(distanceCM*counts_per_cm);
                double remainder = (Math.abs(robot.frontRight.getCurrentPosition() - NEW_upright_target));
                double rampdistance = Math.abs(rampupPercentage*distance);
                double distancetraveled = Math.abs(robot.frontRight.getCurrentPosition()- FRstartEncoder);


                if (distancetraveled < rampdistance ){
                    telemetry.addLine("speeding up");
                    telemetry.addData("distance", distance);
                    telemetry.addData("initial encoder value", FRstartEncoder);
                    telemetry.addData("distance traveled", distancetraveled);
                    telemetry.addData("current position", robot.towerMotor.getCurrentPosition());
                    telemetry.addData("ramp distance", rampdistance);
                    telemetry.addData("remainder", (distance - distancetraveled));
                    telemetry.update();
                    double rampRatio = distancetraveled/rampdistance;
                    robot.towerMotor.setPower(Range.clip((Math.abs(speed*rampRatio)),0.2,speed));

                }

            }
            telemetry.addLine("stop");
            telemetry.update();
            robot.towerMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.towerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }
//----------------------------------------------------------------------------------------------
// Telemetry for IMU
//----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                });


    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}
















