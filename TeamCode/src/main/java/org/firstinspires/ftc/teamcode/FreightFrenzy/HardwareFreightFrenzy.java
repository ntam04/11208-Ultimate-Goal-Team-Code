package org.firstinspires.ftc.teamcode.FreightFrenzy;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareFreightFrenzy {

    public DcMotorEx front_right_motor;
    public DcMotorEx back_right_motor;
    public DcMotorEx front_left_motor;
    public DcMotorEx back_left_motor;
    public DcMotorEx extension_motor_1;
    public DcMotorEx extension_motor_2;
    public DcMotorEx lift_motor;
    public DcMotorEx duck_wheel;

    public Servo drop_servo_1;
    public Servo drop_servo_2;
    public CRServo intake_servo;
    public Servo outtake_servo;



    HardwareMap hwMap;

    public HardwareFreightFrenzy() {

    }

    public void init(HardwareMap ahwMap){

        hwMap = ahwMap;

        //AndyMark motors spin clockwise with positive power (by default)
        //All other motors spin counterclockwise with positive power (by default)

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

        //Set up extension motors
        extension_motor_1 = hwMap.get(DcMotorEx.class, "extension_motor_1");
        extension_motor_1.setDirection(DcMotorSimple.Direction.FORWARD);
        extension_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension_motor_1.setPower(0);

        extension_motor_2 = hwMap.get(DcMotorEx.class, "extension_motor_2");
        extension_motor_2.setDirection(DcMotorSimple.Direction.REVERSE);
        extension_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension_motor_2.setPower(0);

        //Set up lift motor
        lift_motor = hwMap.get(DcMotorEx.class, "lift_motor");
        lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor.setPower(0);

        //drop_servos
        //1 = intake position
        //0.2 = auto init position
        //0.28 = handoff position
        drop_servo_1 = hwMap.get(Servo.class, "drop_servo_1");

        drop_servo_2 = hwMap.get(Servo.class, "drop_servo_2");
        drop_servo_2.setDirection(Servo.Direction.REVERSE);

        //intake_servo
        intake_servo = hwMap.get(CRServo.class, "intake_servo");

        //outtake_servo
        outtake_servo = hwMap.get(Servo.class, "outtake_servo");
        outtake_servo.setDirection(Servo.Direction.REVERSE);

        //duck_wheel
        duck_wheel = hwMap.get(DcMotorEx.class, "duck_wheel");
        duck_wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        duck_wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        duck_wheel.setPower(0);

    }

}
