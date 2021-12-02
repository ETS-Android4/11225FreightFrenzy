package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.enums.ParkingMethod;
import org.firstinspires.ftc.teamcode.autonomous.enums.Position;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class BlueTrajectoryGenerator extends TrajectoryGenerator {
    Position position;
    ParkingMethod parkingMethod;

    public BlueTrajectoryGenerator(SampleMecanumDrive drive, Position position, ParkingMethod parkingMethod) {
        super(drive);
        this.position = position;
        this.parkingMethod = parkingMethod;
    }

    public ArrayList<ArrayList<Trajectory>> generateTrajectories() {
        // TODO generate trajectories based on position and color
        return null;
    }
}
