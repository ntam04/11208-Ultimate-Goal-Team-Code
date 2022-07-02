package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareBasic {

    public DcMotorEx front_right_motor;
    public DcMotorEx back_right_motor;
    public DcMotorEx front_left_motor;
    public DcMotorEx back_left_motor;


    HardwareMap hwMap;

    public HardwareBasic() {

    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //Initialize motors
        front_right_motor = hwMap.get(DcMotorEx.class, "front_right_motor");
        back_right_motor = hwMap.get(DcMotorEx.class, "back_right_motor");
        front_left_motor = hwMap.get(DcMotorEx.class, "front_left_motor");
        back_left_motor = hwMap.get(DcMotorEx.class, "back_left_motor");

        //Reverse direction
        front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set motor powers to 0
        front_right_motor.setPower(0);
        back_right_motor.setPower(0);
        front_left_motor.setPower(0);
        back_left_motor.setPower(0);

        //Set mode to RUN_USING_ENCODERS
        front_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set zero power behavior to 'brake'
        //front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
