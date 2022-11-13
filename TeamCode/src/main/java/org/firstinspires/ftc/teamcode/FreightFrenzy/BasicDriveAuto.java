package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="Basic Auto")
//@Disabled
public class BasicDriveAuto extends LinearOpMode {

    /*
    Calls hardware from hardware function
     */
    HardwareBasic hardwareBasic = new HardwareBasic();


    @Override
    public void runOpMode() {

        //Initializes hardware through hardware function
        hardwareBasic.init(hardwareMap);

        //Calls SampleMecanumDrive which allows the utilization of trajectoryBuilder
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Sets starting position to (-63, -32) with a starting heading of 0 degrees
        Pose2d startPose = new Pose2d(-0, -0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory driveForward = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-15, 15),0)
                .build();

        Trajectory driveBackward = drive.trajectoryBuilder(driveForward.end())
                .splineTo(new Vector2d(0,0),0)
                .build();

        Trajectory DrSi = drive.trajectoryBuilder(driveBackward.end())
                .strafeTo(new Vector2d(-25, 0))
                .build();

        Trajectory Drivele = drive.trajectoryBuilder(DrSi.end())
                .splineTo(new Vector2d(-16,10),0)
                .build();

        Trajectory drivefrr = drive.trajectoryBuilder(Drivele.end())
                .splineTo(new Vector2d(-40, 16), 2)
                .build();

        waitForStart();

        drive.followTrajectory(driveForward);
        drive.followTrajectory(DrSi);
        drive.followTrajectory((Drivele));
        drive.followTrajectory((drivefrr));
    }
}