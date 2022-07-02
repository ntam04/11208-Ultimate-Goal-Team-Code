package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic Drive Train")
public class BasicDriveTrain extends LinearOpMode {

    HardwareBasic hardwareBasic = new HardwareBasic();

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareBasic.init(hardwareMap);

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

        waitForStart();

        while (opModeIsActive()) {

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
            hardwareBasic.front_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardwareBasic.back_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardwareBasic.front_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardwareBasic.back_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);

            if (gamepad1.a) {
                hardwareBasic.front_right_motor.setTargetPosition(0);
                hardwareBasic.back_right_motor.setTargetPosition(0);
                hardwareBasic.front_left_motor.setTargetPosition(0);
                hardwareBasic.back_left_motor.setTargetPosition(0);
                hardwareBasic.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareBasic.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareBasic.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareBasic.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hardwareBasic.front_right_motor.setPower(1);
                hardwareBasic.back_right_motor.setPower(1);
                hardwareBasic.front_left_motor.setPower(1);
                hardwareBasic.back_left_motor.setPower(1);
                sleep(1000);
                hardwareBasic.front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardwareBasic.back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardwareBasic.front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hardwareBasic.back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            while (hardwareBasic.back_left_motor.isBusy()) {
                telemetry.addData("Encoder", hardwareBasic.front_right_motor.getCurrentPosition());
                telemetry.addData("Encoder", hardwareBasic.back_right_motor.getCurrentPosition());
                telemetry.addData("Encoder", hardwareBasic.front_left_motor.getCurrentPosition());
                telemetry.addData("Encoder", hardwareBasic.back_left_motor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
