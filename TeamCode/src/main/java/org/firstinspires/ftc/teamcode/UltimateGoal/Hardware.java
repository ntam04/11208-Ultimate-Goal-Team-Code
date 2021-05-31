package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    public DcMotor front_right_motor;
    public DcMotor back_right_motor;
    public DcMotor front_left_motor;
    public DcMotor back_left_motor;
    public DcMotor intake_motor;
    public DcMotorEx launcher_motor_1;
    public DcMotorEx launcher_motor_2;
    public DcMotor arm_motor;
    public Servo feeder_servo;
    public Servo grabber_servo;


    HardwareMap hwMap;

    public Hardware() {

    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        //Initialize motors
        front_right_motor = hwMap.get(DcMotor.class, "front_right_motor");
        back_right_motor = hwMap.get(DcMotor.class, "back_right_motor");
        front_left_motor = hwMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hwMap.get(DcMotor.class, "back_left_motor");

        //Reverse direction
        front_right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        back_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        front_left_motor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        //Set zero power behavior to 'brake'
        front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set servos
        feeder_servo = hwMap.get(Servo.class, "feeder_servo");
        feeder_servo.setPosition(0);

        //Set up launcher motors
        launcher_motor_1 = hwMap.get(DcMotorEx.class, "launcher_motor_1");
        launcher_motor_1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher_motor_1.setPower(0);

        launcher_motor_2 = hwMap.get(DcMotorEx.class, "launcher_motor_2");
        launcher_motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher_motor_2.setPower(0);

        //Set up intake motor
        intake_motor = hwMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Set up arm
        arm_motor = hwMap.get(DcMotor.class, "arm_motor");
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber_servo = hwMap.get(Servo.class, "grabber_servo");
        grabber_servo.setDirection(Servo.Direction.REVERSE);
        grabber_servo.setPosition(0.5);

    }
}
