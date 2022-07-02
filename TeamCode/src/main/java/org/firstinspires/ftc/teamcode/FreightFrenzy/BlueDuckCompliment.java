package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.TeamMarkerDetectorBlue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Blue Duck Compliment")
public class BlueDuckCompliment extends LinearOpMode {

    HardwareFreightFrenzy hardwareFreightFrenzy = new HardwareFreightFrenzy();
    OpenCvCamera Robot;

    @Override
    public void runOpMode() {

        hardwareFreightFrenzy.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Robot = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        TeamMarkerDetectorBlue detector = new TeamMarkerDetectorBlue(telemetry);
        Robot.setPipeline(detector);

        Robot.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Robot.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }
        });

        //Specifically initialize hardware devices
        //hardwareFreightFrenzy.extension_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //hardwareFreightFrenzy.extension_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardwareFreightFrenzy.outtake_servo.setPosition(0.25);
        hardwareFreightFrenzy.drop_servo_1.setPosition(0.2);
        hardwareFreightFrenzy.drop_servo_2.setPosition(0.2);

        //Sets starting position to (-63, -32) with a starting heading of 0 degrees
        Pose2d startPose = new Pose2d(-38, 62, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory leaveWall = drive.trajectoryBuilder(startPose)
                .back(4)
                .build();

        Trajectory driveToGoal1 = drive.trajectoryBuilder(leaveWall.end())
                .splineToConstantHeading(new Vector2d(-64,48), Math.toRadians(90))
                .build();

        Trajectory driveToGoal2 = drive.trajectoryBuilder(driveToGoal1.end())
                .back(30)
                .build();

        Trajectory driveToGoal3 = drive.trajectoryBuilder(driveToGoal2.end())
                .splineToLinearHeading(new Pose2d(-34,24, Math.toRadians(180)), Math.toRadians(60))
                .build();

        Trajectory driveToDuck1 = drive.trajectoryBuilder(driveToGoal3.end())
                .splineToLinearHeading(new Pose2d(-58,18, Math.toRadians(270)), Math.toRadians(110))
                .build();

        Trajectory driveToDuck2 = drive.trajectoryBuilder(driveToDuck1.end())
                .back(41)
                .addDisplacementMarker(20, () -> {
                    hardwareFreightFrenzy.duck_wheel.setVelocity(-1000);
                })
                .build();

        Trajectory park = drive.trajectoryBuilder(driveToDuck2.end())
                .forward(19)
                .build();

        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                //Spline to shipping hub and reverse to deposit in level 1
                //Spline to duck
                //Park in tape
                drive.followTrajectory(leaveWall);
                drive.followTrajectory(leaveWall);
                drive.followTrajectory(driveToGoal1);
                drive.followTrajectory(driveToGoal2);
                drive.followTrajectory(driveToGoal3);
                hardwareFreightFrenzy.lift_motor.setTargetPosition(800);
                hardwareFreightFrenzy.lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareFreightFrenzy.lift_motor.setPower(1);
                sleep(2000);
                hardwareFreightFrenzy.outtake_servo.setPosition(0.8);
                sleep(1000);
                hardwareFreightFrenzy.outtake_servo.setPosition(0.25);
                sleep(500);
                hardwareFreightFrenzy.lift_motor.setTargetPosition(0);
                hardwareFreightFrenzy.lift_motor.setPower(1);
                sleep(2000);
                drive.followTrajectory(driveToDuck1);
                drive.followTrajectory(driveToDuck2);
                sleep(4000);
                drive.followTrajectory(park);
                telemetry.addData("Position", "Left");
                telemetry.update();
                break;
            case CENTER:
                //Spline to shipping hub and deposit in level 2
                //Spline to duck
                //Park in tape
                drive.followTrajectory(leaveWall);
                drive.followTrajectory(leaveWall);
                drive.followTrajectory(driveToGoal1);
                drive.followTrajectory(driveToGoal2);
                drive.followTrajectory(driveToGoal3);
                hardwareFreightFrenzy.lift_motor.setTargetPosition(1200);
                hardwareFreightFrenzy.lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareFreightFrenzy.lift_motor.setPower(1);
                sleep(2000);
                hardwareFreightFrenzy.outtake_servo.setPosition(0.7);
                sleep(1000);
                hardwareFreightFrenzy.outtake_servo.setPosition(0.25);
                sleep(500);
                hardwareFreightFrenzy.lift_motor.setTargetPosition(0);
                hardwareFreightFrenzy.lift_motor.setPower(1);
                sleep(2000);
                drive.followTrajectory(driveToDuck1);
                drive.followTrajectory(driveToDuck2);
                sleep(4000);
                drive.followTrajectory(park);
                telemetry.addData("Position", "Center");
                telemetry.update();
                break;
            case RIGHT:
                //Spline to shipping hub and deposit in level 2
                //Spline to duck
                //Park in tape
                drive.followTrajectory(leaveWall);
                drive.followTrajectory(driveToGoal1);
                drive.followTrajectory(driveToGoal2);
                drive.followTrajectory(driveToGoal3);
                hardwareFreightFrenzy.lift_motor.setTargetPosition(1400);
                hardwareFreightFrenzy.lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareFreightFrenzy.lift_motor.setPower(1);
                sleep(2000);
                hardwareFreightFrenzy.outtake_servo.setPosition(0.7);
                sleep(1000);
                hardwareFreightFrenzy.outtake_servo.setPosition(0.25);
                sleep(500);
                hardwareFreightFrenzy.lift_motor.setTargetPosition(0);
                hardwareFreightFrenzy.lift_motor.setPower(1);
                sleep(2000);
                drive.followTrajectory(driveToDuck1);
                drive.followTrajectory(driveToDuck2);
                sleep(4000);
                drive.followTrajectory(park);
                telemetry.addData("Position", "Right");
                telemetry.update();
                break;
        }

    }
}