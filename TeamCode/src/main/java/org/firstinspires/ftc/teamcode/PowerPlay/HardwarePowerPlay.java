package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwarePowerPlay {

    public DcMotorEx front_right_motor;
    public DcMotorEx back_right_motor;
    public DcMotorEx front_left_motor;
    public DcMotorEx back_left_motor;
    public DcMotorEx turret_motor;
    public DcMotorEx lift_motor_1;
    public DcMotorEx lift_motor_2;

    public Servo extension_servo;
    public Servo grip_servo;

    HardwareMap hwMap;

    public HardwarePowerPlay() {

    }

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        //AndyMark motors spin clockwise with positive power (by default)
        //All other motors spin counterclockwise with positive power (by default)

        //Initialize drive motors
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

        //Set up turret motor
        turret_motor = hwMap.get(DcMotorEx.class, "turret_motor");
        turret_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        turret_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret_motor.setPower(0);

        //Set up lift motors
        lift_motor_1 = hwMap.get(DcMotorEx.class, "lift_motor_1");
        lift_motor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        lift_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor_1.setPower(0);

        lift_motor_2 = hwMap.get(DcMotorEx.class, "lift_motor_2");
        lift_motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        lift_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor_2.setPower(0);

        //Set up servos
        extension_servo = hwMap.get(Servo.class, "extension_motor");
        grip_servo = hwMap.get(Servo.class, "grip_servo");

    }

}
