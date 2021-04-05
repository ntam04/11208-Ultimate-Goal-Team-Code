package org.firstinspires.ftc.teamcode.LearnJava;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
@Disabled
public abstract class HardwareDevices extends LinearOpMode {

    //Get motors

    DcMotor front_right_motor;
    DcMotor back_right_motor;
    DcMotor front_left_motor;
    DcMotor back_left_motor;

    @Override
    public abstract void runOpMode();{

        front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
        back_right_motor = hardwareMap.dcMotor.get("back_right_motor");
        front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
        back_left_motor = hardwareMap.dcMotor.get("back_left_motor");

        //Reverse left motors

        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);







    }
}
