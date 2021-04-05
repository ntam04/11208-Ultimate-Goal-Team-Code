package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.EasyOpenCVExample;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Standard Auto")
public class Auto extends LinearOpMode {

    /*
    Calls hardware from hardware function
    Calls EasyOpenCVExample for vision
     */
    Hardware hardware = new Hardware();
    OpenCvCamera Robot;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;


    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);

        //Calls SampleMecanumDrive which allows the utilization of trajectoryBuilder
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Initializes webcam for use in OpenCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Robot = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.SkystoneDeterminationPipeline();
        Robot.setPipeline(pipeline);


        //Sets webcam orientation and aspect ratio
        Robot.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Robot.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }
        });


        Pose2d startPose = new Pose2d(-63, -32, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        hardware.feeder_servo.setPosition(0);
        hardware.arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.grabber_servo.setPosition(0.8);
        double servoRestPosition;
        double servoShootPosition;
        servoRestPosition = 0;
        servoShootPosition = 0.2;

        /*
        Builds trajectories:

          - driveToShootingPose:
            Splines to shooting position while maintaining a heading of 0 degrees (forward)

         */
        Trajectory driveToShootingPose = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-20, -20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-8, -32), Math.toRadians(180))
                .build();



        /*
        Trajectories for no rings
         */
        Trajectory driveToZoneA = drive.trajectoryBuilder(driveToShootingPose.end())
                .splineTo(new Vector2d(10,-48), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory pickUp2ndWobbleGoal = drive.trajectoryBuilder(driveToZoneA.end(), true)
                .splineTo(new Vector2d(-30,-40), Math.toRadians(130))
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory pickUp2ndWobbleGoal2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal.end())
                .strafeRight(15)
                .build();

        Trajectory driveToZoneA2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal2.end())
                .splineTo(new Vector2d(0,-45), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();



        /*
        Trajectories for one ring
         */
        Trajectory driveToExtraRing = drive.trajectoryBuilder(driveToShootingPose.end(), true)
                .splineTo(new Vector2d(-19,-36), Math.toRadians(197))
                .addDisplacementMarker(1, () -> {
                    hardware.intake_motor.setPower(-1);
                })
                .build();

        Trajectory driveToZoneB = drive.trajectoryBuilder(driveToShootingPose.end())
                .splineTo(new Vector2d(32,-22), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory pickUp2ndWobbleGoal_2 = drive.trajectoryBuilder(driveToZoneB.end(), true)
                .splineTo(new Vector2d(-47,-20), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory pickUp2ndWobbleGoal2_2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal_2.end())
                .strafeRight(24)
                .build();

        Trajectory positionRobot = drive.trajectoryBuilder(pickUp2ndWobbleGoal2_2.end())
                .strafeLeft(24)
                .build();

        Trajectory driveToZoneB2 = drive.trajectoryBuilder(positionRobot.end())
                .splineTo(new Vector2d(28,-20), Math.toRadians(0))
                .addDisplacementMarker(3, () -> {
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.5);
                })
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory park2 = drive.trajectoryBuilder(driveToZoneB.end())
                .back(28)
                .build();



        /*
        Trajectories for four rings
         */
        Trajectory driveToZoneC = drive.trajectoryBuilder(driveToShootingPose.end())
                .splineTo(new Vector2d(50,-48), Math.toRadians(0))
                .addDisplacementMarker(3, () -> {
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory pickUp2ndWobbleGoal_3 = drive.trajectoryBuilder(driveToZoneC.end(), true)
                .splineTo(new Vector2d(-30,-40), Math.toRadians(130))
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory pickUp2ndWobbleGoal2_3 = drive.trajectoryBuilder(pickUp2ndWobbleGoal_3.end())
                .strafeRight(20)
                .build();

        Trajectory driveToZoneC2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal2_3.end())
                .splineTo(new Vector2d(50,-45), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.5);
                })
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        Trajectory park3 = drive.trajectoryBuilder(driveToZoneC.end())
                .back(50)
                .build();

        /*

        SHOOTING SEQUENCE (COPY FROM HERE):

            hardware.launcher_motor_1.setVelocity(1250);
            hardware.launcher_motor_2.setVelocity(1250);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(2000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(500);
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);

        */

        while (!opModeIsActive()) {

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }


        waitForStart();


        if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE) {
            drive.followTrajectory(driveToShootingPose);
            hardware.launcher_motor_1.setVelocity(1250);
            hardware.launcher_motor_2.setVelocity(1250);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(2000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(500);
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            //drive.followTrajectory(driveToZoneA);
            //drive.followTrajectory(pickUp2ndWobbleGoal);
            //drive.followTrajectory(pickUp2ndWobbleGoal2);
            //hardware.grabber_servo.setPosition(0.8);
            //sleep(500);
            //hardware.arm_motor.setTargetPosition(-10);
            //hardware.arm_motor.setPower(-0.5);
            //drive.followTrajectory(driveToZoneA2);
        }
        else if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE) {
            drive.followTrajectory(driveToShootingPose);
            hardware.launcher_motor_1.setVelocity(1250);
            hardware.launcher_motor_2.setVelocity(1250);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(2000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(500);
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            /*
            drive.followTrajectory(driveToExtraRing);
            sleep(1500);
            hardware.launcher_motor_1.setVelocity(1500);
            hardware.launcher_motor_2.setVelocity(1500);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(500);
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
             */
            drive.followTrajectory(driveToZoneB);
            drive.followTrajectory(pickUp2ndWobbleGoal_2);
            drive.followTrajectory(pickUp2ndWobbleGoal2_2);
            hardware.grabber_servo.setPosition(0.8);
            sleep(1000);
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            drive.followTrajectory(positionRobot);
            drive.followTrajectory(driveToZoneB2);
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            drive.followTrajectory(park2);
        }
        else if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            drive.followTrajectory(driveToShootingPose);
            hardware.launcher_motor_1.setVelocity(1250);
            hardware.launcher_motor_2.setVelocity(1250);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(2000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(500);
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            drive.followTrajectory(driveToZoneC);
            drive.followTrajectory(park3);
        }
        else {
            drive.followTrajectory(driveToShootingPose);
            hardware.launcher_motor_1.setVelocity(1250);
            hardware.launcher_motor_2.setVelocity(1250);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(1000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(1000);
            hardware.intake_motor.setPower(-1);
            sleep(2000);
            hardware.feeder_servo.setPosition(servoShootPosition);
            sleep(500);
            hardware.feeder_servo.setPosition(servoRestPosition);
            sleep(500);
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            drive.followTrajectory(driveToZoneA);
            drive.followTrajectory(pickUp2ndWobbleGoal);
            drive.followTrajectory(pickUp2ndWobbleGoal2);
            hardware.grabber_servo.setPosition(0.8);
            sleep(500);
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            drive.followTrajectory(driveToZoneA2);
        }




    }
}