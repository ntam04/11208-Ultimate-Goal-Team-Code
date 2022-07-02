package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Ultimate Goal TeleOp")
@Disabled
public class UltimateGoalTeleOp extends LinearOpMode {

    //Calls hardware from hardware function
    Hardware hardware = new Hardware();

    @Override
    public void runOpMode() {

        hardware.init(hardwareMap);

        //Calls SampleMecanumDrive which allows the utilization of trajectoryBuilder
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Set motor power variables
        /*
        These variables enable the motor power to be switched between half and full with one button.
        This latched capability will preserve button use.
         */
        double motorPowerInput;
        boolean leftBumperPressed;
        double motorPower;
        double fullPower;
        double halfPower;
        motorPowerInput = 1;
        fullPower = 1;
        halfPower = 0.5;
        motorPower = fullPower;
        leftBumperPressed = false;
        
        //Set intake variables
        /*
        These variables enable the intake to be switched between on and off with one button.
         */
        boolean rightBumperPressed;
        boolean rightTriggerPressed;
        double intakeIn;
        double intakeOff;
        double intakeOut;
        double intakeMode;
        rightBumperPressed = false;
        rightTriggerPressed = false;
        intakeIn = 1;
        intakeOff = 2;
        intakeOut = 3;
        intakeMode = intakeOff;
        
        //Set launcher and feeder servo variables
        /*
        These variables initialize the launcher (mode preparing it to be called),
        servoSleepTime (amount of time the servo requires to move to it's fully extended position),
        servoRestPosition and servoShootPosition (shooting and resting position of the servo),
        and desiredSpeed (the desired RPM which is converted to a usable variable)
         */
        double launcherMode;
        double launcherOn;
        double launcherOff;
        int servoSleepTime;
        double desiredSpeed;
        double desiredRPM;
        double servoRestPosition;
        double servoShootPosition;
        launcherOff = 0;
        launcherOn = 1;
        launcherMode = launcherOff;
        servoSleepTime = 500;
        desiredSpeed = 2500;
        desiredRPM = (desiredSpeed * 28 / 60);
        servoRestPosition = 0;
        servoShootPosition = 0.2;

        /*
        Shooter speeds - 6000 = WAY TOO STRONG
        3000 - Pretty good
        2000 - MID GOAL ******
         */

        hardware.arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //Set starting position
        //drive.setPoseEstimate(new Pose2d(0, -36, Math.toRadians(0)));


        waitForStart();


        while (opModeIsActive()) {

            //Update position
            //drive.update();

            //Set position to estimated current position
            //Pose2d myPose = drive.getPoseEstimate();

            //Build trajectory to drive robot from current position to shooting position
            //Trajectory driveToShootingPose = drive.trajectoryBuilder(myPose)
                    //.splineToLinearHeading(new Pose2d(-5,-36, Math.toRadians(0)), Math.toRadians(0))
                    //.build();

            /*
            Switches motor power to 0.5 when left bumper is pressed
            Switches motor power back to 1 when left bumper is pressed again
             */
            if (gamepad1.left_bumper) {
                if (leftBumperPressed == false) {
                    leftBumperPressed = true;
                    if (motorPower == fullPower) {
                        motorPower = halfPower;
                    }
                    else if (motorPower == halfPower) {
                        motorPower = fullPower;
                    }
                }
            }
            else {
                leftBumperPressed = false;
            }

            //Gives telemetry for motor power "full" or "half"
            if (motorPower == fullPower) {
                motorPowerInput = fullPower;
                telemetry.addData("Motor Power: ", 1);
            }
            else if (motorPower == halfPower) {
                motorPowerInput = halfPower;
                telemetry.addData("Motor Power: ", 0.5);
            }


            //Drives motors
            hardware.front_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardware.back_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardware.front_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardware.back_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);


            //Drives arm motor
            hardware.arm_motor.setPower((0.25)*(gamepad2.left_stick_y));

            //Drives to shooting position if 'a' is pressed
            //if (gamepad1.a) {
                //drive.followTrajectory(driveToShootingPose);
            //}

            /*
            Turns on intake if right bumper is pressed
            Turns off intake if left or right triggers are held
             */
            if (gamepad1.right_bumper) {
                if (rightBumperPressed == false) {
                    rightBumperPressed = true;
                    if (intakeMode == intakeOff) {
                        intakeMode = intakeIn;
                    }
                    else if (intakeMode == intakeIn) {
                        intakeMode = intakeOff;
                    }
                    else if (intakeMode == intakeOut) {
                        intakeMode = intakeIn;
                    }
                }
            }
            else if ((gamepad1.right_trigger >= 0.25)||(gamepad1.left_trigger >= 0.25)) {
                if (rightTriggerPressed == false) {
                    rightTriggerPressed = true;
                    if (intakeMode == intakeOff) {
                        intakeMode = intakeOut;
                    }
                    else if (intakeMode == intakeIn) {
                        intakeMode = intakeOut;
                    }
                    else if (intakeMode == intakeOut) {
                        intakeMode = intakeOut;
                    }
                }
            }
            else rightBumperPressed = false; rightTriggerPressed = false;

            /*
            Turns intake on when right bumper is pressed
            Turns intake off when right bumper is pressed again

            Reverses intake when left or right triggers are held
             */
            if (intakeMode == intakeIn) {
                hardware.intake_motor.setPower(-1);
                telemetry.addData("Intake Mode: ", "On");
            }
            else if (intakeMode == intakeOut) {
                if ((gamepad1.right_trigger >= 0.25)||(gamepad1.left_trigger >= 0.25)) {
                    hardware.intake_motor.setPower(1);
                }
                else hardware.intake_motor.setPower(0);

                telemetry.addData("Intake Mode: ", "Out");
            }
            else if (intakeMode == intakeOff) {
                hardware.intake_motor.setPower(0);
                telemetry.addData("Intake Mode: ", "Off");
            }


            /*
            Turns launcher on when 'x' is pressed
            Turns launcher off when 'y' is pressed
             */
            if (gamepad2.x) {
                hardware.launcher_motor_1.setVelocity(desiredRPM);
                hardware.launcher_motor_2.setVelocity(desiredRPM);
                launcherMode = launcherOn;
            }
            else if (gamepad2.y) {
                hardware.launcher_motor_1.setVelocity(0);
                hardware.launcher_motor_2.setVelocity(0);
                launcherMode = launcherOff;
            }

            //Gives telemetry for launcher mode "on" or "off"
            if (launcherMode == launcherOn) {
                launcherMode = launcherOn;
                telemetry.addData("Launcher Mode: ", "On");
            }
            else if (launcherMode == launcherOff) {
                launcherMode = launcherOff;
                telemetry.addData("Launcher Mode: ", "Off");
            }


            /*
            Shoots three rings automatically when 'a' is pressed
            Will not shoot unless launcher motors are on
            Turns launcher off after shooting

            Shoots rings individually when right bumper is pressed
            Does not turn launcher off after shooting
             */
            if (gamepad2.a) {
                    
                    //First shot
                    hardware.feeder_servo.setPosition(servoShootPosition);
                    sleep(servoSleepTime);
                    hardware.feeder_servo.setPosition(servoRestPosition);
                    sleep(servoSleepTime);

                    //Second shot
                    hardware.feeder_servo.setPosition(servoShootPosition);
                    sleep(servoSleepTime);
                    hardware.feeder_servo.setPosition(servoRestPosition);
                    sleep(servoSleepTime);

                    //Third shot
                    hardware.feeder_servo.setPosition(servoShootPosition);
                    sleep(servoSleepTime);
                    hardware.feeder_servo.setPosition(servoRestPosition);
                    sleep(servoSleepTime);

                    //Turns launcher off
                    hardware.launcher_motor_1.setVelocity(0);
                    hardware.launcher_motor_2.setVelocity(0);

                    launcherMode = launcherOff;
                    telemetry.addData("Launcher Mode: ", launcherMode);
                }
                else if (gamepad2.right_bumper) {

                    //Shoots rings individually
                    hardware.feeder_servo.setPosition(servoShootPosition);
                    sleep(servoSleepTime);
                    hardware.feeder_servo.setPosition(servoRestPosition);
                    sleep(servoSleepTime);
                }

                if (gamepad2.dpad_down) {
                    hardware.grabber_servo.setPosition(0);
                }
                else if (gamepad2.dpad_up) {
                    hardware.grabber_servo.setPosition(0.5);
                }

            telemetry.update();
        }
    }
}
