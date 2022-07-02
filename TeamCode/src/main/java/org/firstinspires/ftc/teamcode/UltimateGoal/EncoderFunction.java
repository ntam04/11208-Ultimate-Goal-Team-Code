package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Encoder Function Test")
@Disabled
public abstract class EncoderFunction extends LinearOpMode {

    Hardware hardware = new Hardware();

    public void encoderFunction(double movementMode, double motorPower, double ticksToMove) throws InterruptedException {
        if (opModeIsActive()) {

            //Set encoder specific modes

            hardware.front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            hardware.front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double forwardMove = 1;
            double backwardMove = 2;
            double rightStrafe = 3;
            double leftStrafe = 4;
            double cwTurn = 5;
            double ccwTurn = 6;
            double forwardDiagonalRightMove = 7;
            double forwardDiagonalLeftMove = 8;
            double backwardDiagonalRightMove = 9;
            double backwardDiagonalLeftMove = 10;

            double direction = movementMode;
            double power = motorPower;
            double distance = ticksToMove;

            telemetry.addData("Movement Mode:", direction);
            telemetry.addData("Motor Power:", power);
            telemetry.addData("Distance:", distance);
            telemetry.update();

            int targetPosition = (int) ticksToMove;

            if (movementMode == forwardMove) {

                hardware.front_right_motor.setTargetPosition(targetPosition);
                hardware.back_right_motor.setTargetPosition(targetPosition);
                hardware.front_left_motor.setTargetPosition(targetPosition);
                hardware.back_left_motor.setTargetPosition(targetPosition);

                hardware.front_right_motor.setPower(motorPower);
                hardware.back_right_motor.setPower(motorPower);
                hardware.front_left_motor.setPower(motorPower);
                hardware.back_left_motor.setPower(motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder=", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_right_encoder=", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder=", hardware.front_left_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder=", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == backwardMove) {

                hardware.front_right_motor.setTargetPosition(-targetPosition);
                hardware.back_right_motor.setTargetPosition(-targetPosition);
                hardware.front_left_motor.setTargetPosition(-targetPosition);
                hardware.back_left_motor.setTargetPosition(-targetPosition);

                hardware.front_right_motor.setPower(-motorPower);
                hardware.back_right_motor.setPower(-motorPower);
                hardware.front_left_motor.setPower(-motorPower);
                hardware.back_left_motor.setPower(-motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder=", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_right_encoder=", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder=", hardware.front_left_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder=", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();

                }
            } else if (movementMode == rightStrafe) {

                hardware.front_right_motor.setTargetPosition(-targetPosition);
                hardware.back_right_motor.setTargetPosition(targetPosition);
                hardware.front_left_motor.setTargetPosition(targetPosition);
                hardware.back_left_motor.setTargetPosition(-targetPosition);

                hardware.front_right_motor.setPower(-motorPower);
                hardware.back_right_motor.setPower(motorPower);
                hardware.front_left_motor.setPower(motorPower);
                hardware.back_left_motor.setPower(-motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder=", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_right_encoder=", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder=", hardware.front_left_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder=", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == leftStrafe) {

                hardware.front_right_motor.setTargetPosition(targetPosition);
                hardware.back_right_motor.setTargetPosition(-targetPosition);
                hardware.front_left_motor.setTargetPosition(-targetPosition);
                hardware.back_left_motor.setTargetPosition(targetPosition);

                hardware.front_right_motor.setPower(motorPower);
                hardware.back_right_motor.setPower(-motorPower);
                hardware.front_left_motor.setPower(-motorPower);
                hardware.back_left_motor.setPower(motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder=", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_right_encoder=", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder=", hardware.front_left_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder=", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == cwTurn) {

                hardware.front_right_motor.setTargetPosition(-targetPosition);
                hardware.back_right_motor.setTargetPosition(-targetPosition);
                hardware.front_left_motor.setTargetPosition(targetPosition);
                hardware.back_left_motor.setTargetPosition(targetPosition);

                hardware.front_right_motor.setPower(-motorPower);
                hardware.back_right_motor.setPower(-motorPower);
                hardware.front_left_motor.setPower(motorPower);
                hardware.back_left_motor.setPower(motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder=", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_right_encoder=", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder=", hardware.front_left_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder=", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == ccwTurn) {

                hardware.front_right_motor.setTargetPosition(targetPosition);
                hardware.back_right_motor.setTargetPosition(targetPosition);
                hardware.front_left_motor.setTargetPosition(-targetPosition);
                hardware.back_left_motor.setTargetPosition(-targetPosition);

                hardware.front_right_motor.setPower(motorPower);
                hardware.back_right_motor.setPower(motorPower);
                hardware.front_left_motor.setPower(-motorPower);
                hardware.back_left_motor.setPower(-motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder=", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_right_encoder=", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder=", hardware.front_left_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder=", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == forwardDiagonalRightMove) {

                hardware.front_right_motor.setTargetPosition(targetPosition);
                hardware.back_left_motor.setTargetPosition(targetPosition);

                hardware.front_right_motor.setPower(motorPower);
                hardware.back_left_motor.setPower(motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == forwardDiagonalLeftMove) {

                hardware.back_right_motor.setTargetPosition(targetPosition);
                hardware.front_left_motor.setTargetPosition(targetPosition);

                hardware.back_right_motor.setPower(motorPower);
                hardware.front_left_motor.setPower(motorPower);

                while (opModeIsActive() & hardware.back_right_motor.isBusy()) {

                    telemetry.addData("back_right_encoder", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder", hardware.front_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == backwardDiagonalRightMove) {

                hardware.front_right_motor.setTargetPosition(-targetPosition);
                hardware.back_left_motor.setTargetPosition(-targetPosition);

                hardware.front_right_motor.setPower(-motorPower);
                hardware.back_left_motor.setPower(-motorPower);

                while (opModeIsActive() & hardware.front_right_motor.isBusy()) {

                    telemetry.addData("front_right_encoder", hardware.front_right_motor.getCurrentPosition());
                    telemetry.addData("back_left_encoder", hardware.back_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            } else if (movementMode == backwardDiagonalLeftMove) {

                hardware.back_right_motor.setTargetPosition(-targetPosition);
                hardware.front_left_motor.setTargetPosition(-targetPosition);

                hardware.back_right_motor.setPower(-motorPower);
                hardware.front_left_motor.setPower(-motorPower);

                while (opModeIsActive() & hardware.back_right_motor.isBusy()) {

                    telemetry.addData("back_right_encoder", hardware.back_right_motor.getCurrentPosition());
                    telemetry.addData("front_left_encoder", hardware.front_left_motor.getCurrentPosition());
                    telemetry.update();
                }
            }
        }
    }



}
