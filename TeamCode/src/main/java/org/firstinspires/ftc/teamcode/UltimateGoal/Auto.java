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

        //Initializes hardware through hardware function
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


        //Sets starting position to (-63, -32) with a starting heading of 0 degrees
        Pose2d startPose = new Pose2d(-63, -32, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        //Sets servo positions, servo position variables, and arm motor behavior
        hardware.feeder_servo.setPosition(0);
        hardware.arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.grabber_servo.setPosition(0.8);
        double servoRestPosition;
        double servoShootPosition;
        servoRestPosition = 0;
        servoShootPosition = 0.2;


        //Drives to shooting position
        Trajectory driveToShootingPose = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-20, -20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-8, -32), Math.toRadians(180))
                .build();



        /*
        Trajectories for no rings
         */

        //Drives from shooting position to Zone A
        Trajectory driveToZoneA = drive.trajectoryBuilder(driveToShootingPose.end())
                .splineTo(new Vector2d(10,-48), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    //Lowers arm
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    //Opens grabber
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        //Drives from Zone A to second wobble goal
        Trajectory pickUp2ndWobbleGoal = drive.trajectoryBuilder(driveToZoneA.end(), true)
                .splineTo(new Vector2d(-30,-40), Math.toRadians(130))
                .addDisplacementMarker(() -> {
                    //Ensures grabber is open
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        //Drives into second wobble goal for accurate pick-up
        Trajectory pickUp2ndWobbleGoal2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal.end())
                .strafeRight(15)
                .build();

        //Drives to Zone A and parks
        Trajectory driveToZoneA2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal2.end())
                .splineTo(new Vector2d(0,-45), Math.toRadians(0))
                .addDisplacementMarker(3, () -> {
                    //Raises arm
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    //Opens grabber
                    hardware.grabber_servo.setPosition(0);
                })
                .build();



        /*
        Trajectories for one ring
         */
        //Drives to extra ring *QUESTIONABLE*
        Trajectory driveToExtraRing = drive.trajectoryBuilder(driveToShootingPose.end(), true)
                .splineTo(new Vector2d(-19,-36), Math.toRadians(197))
                .addDisplacementMarker(1, () -> {
                    //Turns on intake
                    hardware.intake_motor.setPower(-1);
                })
                .build();

        //Drives from shooting position to Zone B
        Trajectory driveToZoneB = drive.trajectoryBuilder(driveToShootingPose.end())
                .splineTo(new Vector2d(32,-22), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    //Lowers arm
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    //Opens grabber
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        //Drives to position to pick up second wobble goal
        Trajectory pickUp2ndWobbleGoal_2 = drive.trajectoryBuilder(driveToZoneB.end(), true)
                .splineTo(new Vector2d(-47,-20), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    //Ensures grabber is open
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        //Strafes to pick up second wobble goal
        Trajectory pickUp2ndWobbleGoal2_2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal_2.end())
                .strafeRight(24)
                .build();

        //Strafes to position
        Trajectory positionRobot = drive.trajectoryBuilder(pickUp2ndWobbleGoal2_2.end())
                .strafeLeft(24)
                .build();

        //Drives to Zone B
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

        //Parks
        Trajectory park2 = drive.trajectoryBuilder(driveToZoneB.end())
                .back(28)
                .build();



        /*
        Trajectories for four rings
         */
        //Drives from shooting position to Zone C
        Trajectory driveToZoneC = drive.trajectoryBuilder(driveToShootingPose.end())
                .splineTo(new Vector2d(50,-48), Math.toRadians(0))
                .addDisplacementMarker(3, () -> {
                    //Lowers arm
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.25);
                })
                .addDisplacementMarker(() -> {
                    //Opens grabber
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        //Drives to position to pick up second wobble goal
        Trajectory pickUp2ndWobbleGoal_3 = drive.trajectoryBuilder(driveToZoneC.end(), true)
                .splineTo(new Vector2d(0,-20), Math.toRadians(180))
                .splineTo(new Vector2d(-47,-20), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    //Ensures grabber is open
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        //Strafes to pick up second wobble goal
        Trajectory pickUp2ndWobbleGoal2_3 = drive.trajectoryBuilder(pickUp2ndWobbleGoal_3.end())
                .strafeRight(24)
                .build();

        //Strafes to position
        Trajectory positionRobot2 = drive.trajectoryBuilder(pickUp2ndWobbleGoal2_3.end())
                .strafeLeft(24)
                .build();

        //Drives to Zone C
        Trajectory driveToZoneC2 = drive.trajectoryBuilder(positionRobot2.end())
                .splineTo(new Vector2d(50,-46), Math.toRadians(0))
                .addDisplacementMarker(3, () -> {
                    hardware.arm_motor.setTargetPosition(10);
                    hardware.arm_motor.setPower(0.5);
                })
                .addDisplacementMarker(() -> {
                    hardware.grabber_servo.setPosition(0);
                })
                .build();

        //Parks (NOTE: doesn't have time so stalls in Zone C after depositing second wobble goal)
        Trajectory park3 = drive.trajectoryBuilder(driveToZoneC2.end())
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
            //Drives to shooting position and shoots three rings
            //Turns on shooter
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
            //Turns off shooter
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            //Drives to deposit wobble goal
            drive.followTrajectory(driveToZoneA);
            //Drives to pick up second wobble goal
            drive.followTrajectory(pickUp2ndWobbleGoal);
            drive.followTrajectory(pickUp2ndWobbleGoal2);
            //Closes grabber
            hardware.grabber_servo.setPosition(0.8);
            sleep(500);
            //Lifts arm
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            //Drives to deposit second wobble goal and parks
            drive.followTrajectory(driveToZoneA2);
        }
        else if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE) {
            //Drives to shooting position and shoots three rings
            //Turns on shooter
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
            //Turns off shooter
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            /*
            EXTRA RING SEQUENCE:

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
            //Drives to deposit wobble goal
            drive.followTrajectory(driveToZoneB);
            //Drives to pick up second wobble goal
            drive.followTrajectory(pickUp2ndWobbleGoal_2);
            drive.followTrajectory(pickUp2ndWobbleGoal2_2);
            //Closes grabber
            hardware.grabber_servo.setPosition(0.8);
            sleep(1000);
            //Lifts arm
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            //Drives to deposit second wobble goal
            drive.followTrajectory(positionRobot);
            drive.followTrajectory(driveToZoneB2);
            //Lowers arm
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            //Parks
            drive.followTrajectory(park2);
        }
        else if (pipeline.position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR) {
            //Drives to shooting position and shoots three rings
            //Turns on shooter
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
            //Turns off shooter
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            //Drives to deposit wobble goal
            drive.followTrajectory(driveToZoneC);
            //Drives to pick up second wobble goal
            drive.followTrajectory(pickUp2ndWobbleGoal_3);
            drive.followTrajectory(pickUp2ndWobbleGoal2_3);
            //Closes grabber
            hardware.grabber_servo.setPosition(0.8);
            sleep(1000);
            //Lifts arm
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            //Drives to deposit second wobble goal
            drive.followTrajectory(positionRobot2);
            drive.followTrajectory(driveToZoneC2);
            //Lifts arm
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            //Parks (NOTE: doesn't have time so stalls in Zone C after depositing second wobble goal)
            drive.followTrajectory(park3);
        }
        else {
            //Drives to shooting position and shoots three rings
            //Turns on shooter
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
            //Turns off shooter
            hardware.launcher_motor_1.setVelocity(0);
            hardware.launcher_motor_2.setVelocity(0);
            hardware.intake_motor.setPower(0);
            //Drives to deposit wobble goal
            drive.followTrajectory(driveToZoneA);
            //Drives to pick up second wobble goal
            drive.followTrajectory(pickUp2ndWobbleGoal);
            drive.followTrajectory(pickUp2ndWobbleGoal2);
            //Closes grabber
            hardware.grabber_servo.setPosition(0.8);
            sleep(500);
            //Lifts arm
            hardware.arm_motor.setTargetPosition(-10);
            hardware.arm_motor.setPower(-0.5);
            //Drives to deposit second wobble goal and parks
            drive.followTrajectory(driveToZoneA2);
        }
    }
}