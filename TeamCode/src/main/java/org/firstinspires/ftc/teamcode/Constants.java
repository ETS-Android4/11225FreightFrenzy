package org.firstinspires.ftc.teamcode;

/**
* This is a class that holds all necessary constants for the robot. Put "magic numbers" here.
* This class cannot be instantiated. Do not inherit from it.
* To add a new constant, simply declare a new public final static variable and assign the
* appropriate value.
*/
public abstract class Constants {
    public final static double dumpPosition = 0.0;
    public final static double collectPosition = 0.65;

    public final static double tseArmCollectPosition = 1.0;
    public final static double tseArmDumpPosition = 0.65;

    public final static double tseRodCollectPosition = 1.0;
    public final static double tseRodDumpPosition = 0.65;

    public final static double fastMultiplier = 1.0;
    public final static double normalMultiplier = 0.6;
    public final static double slowMultiplier = 0.3;

    // TODO tune this speed, may be .7
    public final static double towerWheelSpeed = .67;
}
