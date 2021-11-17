package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.autonomous.enums.Position;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

import java.util.ArrayList;

public class BlueTrajectoryGenerator extends TrajectoryGenerator {
    Position position;

    public BlueTrajectoryGenerator(SampleMecanumDrive drive, Position position) {
        super(drive);
        this.position = position;
    }

    public ArrayList<ArrayList<Trajectory>> generateTrajectories() {
        // TODO generate trajectories based on position and color
        return null;
    }
}
