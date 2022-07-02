package org.firstinspires.ftc.teamcode.UltimateGoal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "SkystoneAutoCloseParkRed (Blocks to Java)")
@Disabled
public class SkystoneAutoCloseParkRed extends LinearOpMode {

  private DcMotor front_right_motor;
  private DcMotor front_left_motor;
  private DcMotor back_right_motor;
  private DcMotor back_left_motor;
  private DcMotor arm_motor;
  private DcMotor right_intakemotorAsDcMotor;
  private DcMotor left_intakemotorAsDcMotor;
  private Servo marker_servo;
  private Servo side_arm;

  double MovementMode_Forward;
  double NR40TPR;
  double MovementMode_Backward;
  double MovementMode_Right;
  double MovementMode_Left;
  double MovementMode_CW;
  double MovementMode_CCW;


  private void SetUpAuto() {
    front_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    front_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
    back_right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    back_left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
    front_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    front_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    back_right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    back_left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    front_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    front_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    back_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    back_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //arm_motor.setDirection(DcMotorSimple.Direction.FORWARD);
    //right_intakemotorAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    //left_intakemotorAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    //arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //right_intakemotorAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //left_intakemotorAsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //right_intakemotorAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //left_intakemotorAsDcMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //marker_servo.setPosition(1);
    //side_arm.setPosition(0.55);
    MovementMode_Forward = 1;
    MovementMode_Backward = 2;
    MovementMode_Right = 3;
    MovementMode_Left = 4;
    MovementMode_CCW = 5;
    MovementMode_CW = 6;
    NR40TPR = 1120;
  }


  @Override
  public void runOpMode() {
    front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
    front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
    back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");
    back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
    //arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");
    //right_intakemotorAsDcMotor = hardwareMap.get(DcMotor.class, "right_intakemotorAsDcMotor");
    //left_intakemotorAsDcMotor = hardwareMap.get(DcMotor.class, "left_intakemotorAsDcMotor");
    //marker_servo = hardwareMap.get(Servo.class, "marker_servo");
    //side_arm = hardwareMap.get(Servo.class, "side_arm");

    SetUpAuto();

    waitForStart();

    if (opModeIsActive()) {

      //Encoder_Move(MovementMode_Forward, 0.5, 5);
      telemetry.addData("Moving Forward...", "");
      telemetry.update();
    }
  }


  private void Encoder_Move(double Movement_Mode, double Motor_Power, double Distance_to_Travel) {
    int TicksToMove;

    front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    front_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    back_left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //((DcMotorEx) front_right_motor).setTargetPositionTolerance(20);
    //((DcMotorEx) front_left_motor).setTargetPositionTolerance(20);
    //((DcMotorEx) back_right_motor).setTargetPositionTolerance(20);
    //((DcMotorEx) back_left_motor).setTargetPositionTolerance(20);
    //((DcMotorEx) front_right_motor).setVelocityPIDFCoefficients(12.5, 3, 0, 10);
    // ((DcMotorEx) back_right_motor).setVelocityPIDFCoefficients(12.5, 3, 0, 10);
    // ((DcMotorEx) back_left_motor).setVelocityPIDFCoefficients(12.5, 3, 0, 10);
    // ((DcMotorEx) front_left_motor).setVelocityPIDFCoefficients(12.5, 3, 0, 10);
    TicksToMove = (int) ((Distance_to_Travel / (4 * Math.PI)) * NR40TPR);
    if (Movement_Mode == MovementMode_Forward) {
      front_right_motor.setTargetPosition(TicksToMove);
      front_left_motor.setTargetPosition(TicksToMove);
      back_right_motor.setTargetPosition(TicksToMove);
      back_left_motor.setTargetPosition(TicksToMove);
      while (front_right_motor.isBusy() || back_right_motor.isBusy() || back_left_motor.isBusy() || front_left_motor.isBusy()) {
        front_right_motor.setPower(Motor_Power);
        front_left_motor.setPower(Motor_Power);
        back_right_motor.setPower(Motor_Power);
        back_left_motor.setPower(Motor_Power);
      }
    } else if (Movement_Mode == MovementMode_Backward) {
      front_right_motor.setTargetPosition(-TicksToMove);
      front_left_motor.setTargetPosition(-TicksToMove);
      back_right_motor.setTargetPosition(-TicksToMove);
      back_left_motor.setTargetPosition(-TicksToMove);
      while (front_right_motor.isBusy() || back_right_motor.isBusy() || back_left_motor.isBusy() || front_left_motor.isBusy()) {
        front_right_motor.setPower(-Motor_Power);
        front_left_motor.setPower(-Motor_Power);
        back_right_motor.setPower(-Motor_Power);
        back_left_motor.setPower(-Motor_Power);
      }
    } else if (Movement_Mode == MovementMode_Right) {
      front_right_motor.setTargetPosition(-TicksToMove);
      front_left_motor.setTargetPosition(TicksToMove);
      back_right_motor.setTargetPosition(TicksToMove);
      back_left_motor.setTargetPosition(-TicksToMove);
      while (front_right_motor.isBusy() || back_right_motor.isBusy() || back_left_motor.isBusy() || front_left_motor.isBusy()) {
        front_right_motor.setPower(-Motor_Power);
        front_left_motor.setPower(Motor_Power);
        back_right_motor.setPower(Motor_Power);
        back_left_motor.setPower(-Motor_Power);
      }
    } else if (Movement_Mode == MovementMode_Left) {
      front_right_motor.setTargetPosition(TicksToMove);
      front_left_motor.setTargetPosition(-TicksToMove);
      back_right_motor.setTargetPosition(-TicksToMove);
      back_left_motor.setTargetPosition(TicksToMove);
      while (front_right_motor.isBusy() || back_right_motor.isBusy() || back_left_motor.isBusy() || front_left_motor.isBusy()) {
        front_right_motor.setPower(Motor_Power);
        front_left_motor.setPower(-Motor_Power);
        back_right_motor.setPower(-Motor_Power);
        back_left_motor.setPower(Motor_Power);
      }
    } else if (Movement_Mode == MovementMode_CW) {
      front_right_motor.setTargetPosition(-TicksToMove);
      front_left_motor.setTargetPosition(TicksToMove);
      back_right_motor.setTargetPosition(-TicksToMove);
      back_left_motor.setTargetPosition(TicksToMove);
      while (front_right_motor.isBusy() || back_right_motor.isBusy() || back_left_motor.isBusy() || front_left_motor.isBusy()) {
        front_right_motor.setPower(-Motor_Power);
        front_left_motor.setPower(Motor_Power);
        back_right_motor.setPower(-Motor_Power);
        back_left_motor.setPower(Motor_Power);
      }
    } else if (Movement_Mode == MovementMode_CCW) {
      front_right_motor.setTargetPosition(TicksToMove);
      front_left_motor.setTargetPosition(-TicksToMove);
      back_right_motor.setTargetPosition(TicksToMove);
      back_left_motor.setTargetPosition(-TicksToMove);
      front_right_motor.setPower(Motor_Power);
      front_left_motor.setPower(-Motor_Power);
      back_right_motor.setPower(Motor_Power);
      back_left_motor.setPower(-Motor_Power);
      while (front_right_motor.isBusy() || back_right_motor.isBusy() || back_left_motor.isBusy() || front_left_motor.isBusy()) {
      }
    }
    front_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    front_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    back_left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }
}
