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

    public final static double tseArmInitPosition = 0.08;
    public final static double tseArmActivePosition = 0.46;

    public final static double tseRodInitPosition = 0.0;
    public final static double tseRodActivePosition = 0.45;

    public final static double fastMultiplier = 1.0;
    public final static double normalMultiplier = 0.6;
    public final static double slowMultiplier = 0.3;

    // TODO tune this speed, decrease
    public final static double towerWheelSpeed = .6;
}