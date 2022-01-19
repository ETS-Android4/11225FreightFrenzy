package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
* This class handles control dispatching. Simply access any of these variables to get the
* status of that action's corresponding control.
* DO NOT instantiate this class. It won't let you. All needed variables and methods are static.
* Also do not INHERIT FROM this class.
* AT THE BEGINNING of the teleop loop call this line of code: ControlConfig.update(gamepad1, gamepad2);
* IF YOU DON'T DO THIS THE CONTROLS WILL NOT BE REFRESHED.
* To add an extra action:
* * Define a public static class variable with the appropriate data type
* * In the update method, assign to the class variable the appropriate data
*/
public abstract class ControlConfig {
    // Movement
    public static double forward;
    public static double backward;
    public static double right;
    public static double left;
    public static double clockwise;

    public static boolean slow;
    public static boolean fast;

    // Peripherals
    public static boolean collectWheel;
    public static boolean unCollectWheel;

    public static boolean liftBucket;
    public static boolean lowerBucket;

    public static boolean duckWheelForward;
    public static boolean duckWheelBackward;

    public static boolean dumpServo;
    public static boolean collectServo;

    public static boolean tseRodServo;
    public static boolean tseArmServo;

    public static void update(Gamepad pad1, Gamepad pad2) {
        // Update movement controls;
        forward = -pad1.left_stick_y;
        backward = pad1.left_stick_y;
        right = pad1.left_stick_x;
        left = -pad1.left_stick_x;
        clockwise = pad1.right_stick_x;

        fast = pad1.left_bumper;
        slow = pad1.right_bumper;

        // Update peripheral controls
        collectWheel = pad1.left_trigger > .1;
        unCollectWheel = pad1.right_trigger > .1;

        liftBucket = pad2.dpad_up;
        lowerBucket = pad2.dpad_down;

        duckWheelForward = pad2.right_bumper;
        duckWheelBackward = pad2.left_bumper;

        dumpServo = pad2.y;
        collectServo = pad2.a;
        
        tseRodServo = pad1.a;
        tseArmServo = pad1.y;
        
    }
}
