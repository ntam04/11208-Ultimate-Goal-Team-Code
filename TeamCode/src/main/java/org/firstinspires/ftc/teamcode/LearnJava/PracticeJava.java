package org.firstinspires.ftc.teamcode.LearnJava;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Drive Forward", group="Exercises")
@Disabled
public abstract class PracticeJava extends LinearOpMode
{

    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public abstract void runOpMode() throws InterruptedException;
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        leftMotor.setPower(0.25);
        rightMotor.setPower(0.25);

        sleep(2000);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        leftMotor.setPower(1);
        rightMotor.setPower(1);





    }
}
