package org.firstinspires.ftc.teamcode.LearnJava;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.UltimateGoal.Hardware;

@TeleOp(name="Mecanum Drive")
//@Disabled
public class MecanumDrive extends LinearOpMode {

    Hardware hardware = new Hardware();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        hardware.init(hardwareMap);

        //Set motor power to 1
        double motorPowerInput;
        motorPowerInput = 1;
        
        waitForStart();

        while (opModeIsActive()){

            double yJoystickPower;
            yJoystickPower = -(gamepad1.left_stick_y + gamepad1.right_stick_y);

            double xJoystickPower;
            xJoystickPower = (gamepad1.left_stick_x + gamepad1.right_stick_x);

            //Control motor power
            if (gamepad1.right_bumper){

                motorPowerInput = 1;
            }

            else if (gamepad1.left_bumper){

                motorPowerInput = 0.5;
            }

            //Display motor powers
            telemetry.addData("Motor Power", motorPowerInput);

            //Set motors
            hardware.front_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardware.back_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardware.front_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);
            hardware.back_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1)*motorPowerInput);

            if (gamepad1.a) {
                telemetry.addData("Pressed:", "a");
            }

            if (gamepad1.b) {
                telemetry.addData("Pressed:", "b");
            }

            if (gamepad1.x) {
                telemetry.addData("Pressed:", "x");
            }

            if (gamepad1.y) {
                telemetry.addData("Pressed:", "y");
            }

            if (gamepad1.dpad_down) {
                telemetry.addData("Pressed:", "dPad_down");
            }

            if (gamepad1.dpad_up) {
                telemetry.addData("Pressed:", "dPad_up");
            }

            if (yJoystickPower >= 1) {
                telemetry.addData("Movement Mode:", "Forward");
                telemetry.addData("Power:", yJoystickPower);
            }

            if (xJoystickPower >= 1) {
                telemetry.addData("Movement Mode:", "Strafe Right");
                telemetry.addData("Power:", xJoystickPower);
            }


            telemetry.update();


        }
    }
}
