package org.firstinspires.ftc.teamcode.PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "PowerPlay TeleOp")
public class PowerPlayTeleOp extends LinearOpMode {

    HardwarePowerPlay hardwarePowerPlay = new HardwarePowerPlay();

    //High Goal Left
    public enum HighGoalLeft {
        LIFT_UP_1,
        LIFT_TURN_1,
        LIFT_TRANSITION_1
    }

    //Mid Goal Left
    public enum MidGoalLeft {
        LIFT_UP_2,
        LIFT_TURN_2,
        LIFT_TRANSITION_2
    }

    //Low Goal Right
    public enum LowGoalLeft {
        LIFT_UP_3,
        LIFT_TURN_3,
        LIFT_TRANSITION_3
    }

    //Ground Goal Left
    public enum GroundGoalLeft {
        LIFT_UP_4,
        LIFT_TURN_4,
        LIFT_TRANSITION_4
    }

    //Retract Lift
    public enum RetractLift {
        LIFT_SCORE,
        LIFT_RETRACT,
        LIFT_DOWN,
        LIFT_RESET
    }

    //Retract Lift (for Ground Goal)
    public enum RetractLift2 {
        LIFT_SCORE_2,
        LIFT_PARTIAL_UP_2,
        LIFT_RETRACT_2,
        LIFT_DOWN_2,
        LIFT_RESET_2
    }

    HighGoalLeft highGoalLeft = HighGoalLeft.LIFT_UP_1;
    MidGoalLeft midGoalLeft = MidGoalLeft.LIFT_UP_2;
    LowGoalLeft lowGoalLeft = LowGoalLeft.LIFT_UP_3;
    GroundGoalLeft groundGoalLeft = GroundGoalLeft.LIFT_UP_4;
    RetractLift retractLift = RetractLift.LIFT_SCORE;
    RetractLift2 retractLift2 = RetractLift2.LIFT_SCORE_2;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwarePowerPlay.init(hardwareMap);

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

        //Lift variables
        double liftPosition;
        double liftOff;
        double liftWait;
        double liftDown;
        liftOff = 1;
        liftWait = 2;
        liftDown = 3;
        liftPosition = liftOff;

        boolean dPadUpPressed;
        double liftUp1;
        dPadUpPressed = false;
        liftUp1 = 1;

        boolean dPadLeftPressed;
        double liftUp2;
        dPadLeftPressed = false;
        liftUp2 = 1;

        boolean dPadRightPressed;
        double liftUp3;
        dPadRightPressed = false;
        liftUp3 = 1;

        boolean dPadDownPressed;
        double liftUp4;
        double liftDown2;
        dPadDownPressed = false;
        liftUp4 = 1;
        liftDown2 = 2;

        //EXPERIMENTAL VALUES!!!!
        final int LIFT_HIGH;
        final int LIFT_MID;
        final int LIFT_LOW;
        final int LIFT_GROUND;
        final int LIFT_PARTIAL;
        final int LIFT_INTAKE;
        final int NINETY_DEGREES_LEFT;
        final int NINETY_DEGREES_RIGHT;
        final int INTAKE_ROT;
        double INTAKE_POS;
        double EXTEND_POS;
        double CLOSE_GRIP;
        double OPEN_GRIP;
        double SCORE_TIME;
        double RETRACT_TIME;
        LIFT_HIGH = 1500;
        LIFT_MID = 1300;
        LIFT_LOW = 1100;
        LIFT_GROUND = 500;
        LIFT_PARTIAL = 900;
        LIFT_INTAKE = 0;
        NINETY_DEGREES_LEFT = -800;
        NINETY_DEGREES_RIGHT = 800;
        INTAKE_ROT = 0;
        INTAKE_POS = 0.3;
        EXTEND_POS = 0.8;
        CLOSE_GRIP = 0.2;
        OPEN_GRIP = 0.6;
        SCORE_TIME = 0.5;
        RETRACT_TIME = 0.5;
        ElapsedTime scoreTimer = new ElapsedTime();
        ElapsedTime retractTimer = new ElapsedTime();

        scoreTimer.reset();
        retractTimer.reset();

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
            } else if (motorPower == halfPower) {
                motorPowerInput = halfPower;
                telemetry.addData("Motor Power: ", 0.5);
            }

            //Drives motors
            hardwarePowerPlay.front_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);
            hardwarePowerPlay.back_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);
            hardwarePowerPlay.front_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);
            hardwarePowerPlay.back_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);


            //Lift presets
            if (gamepad2.dpad_up) {
                if (dPadUpPressed == false) {
                    dPadUpPressed = true;
                    if (liftPosition == liftOff) {
                        liftPosition = liftUp1;
                    }
                    else if (liftPosition == liftWait) {
                        liftPosition = liftDown;
                    }
                }
            }
            else dPadUpPressed = false;

            switch (highGoalLeft) {
                case LIFT_UP_1:
                    if (liftPosition == liftUp1) {
                        hardwarePowerPlay.lift_motor_1.setTargetPosition(LIFT_HIGH);
                        hardwarePowerPlay.lift_motor_2.setTargetPosition(LIFT_HIGH);
                        hardwarePowerPlay.lift_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_1.setPower(1);
                        hardwarePowerPlay.lift_motor_2.setPower(1);
                        highGoalLeft = HighGoalLeft.LIFT_TURN_1;
                    }
                    break;
                case LIFT_TURN_1:
                    if (Math.abs(hardwarePowerPlay.lift_motor_1.getCurrentPosition() - LIFT_HIGH) < 10) {
                        hardwarePowerPlay.extension_servo.setPosition(EXTEND_POS);
                        hardwarePowerPlay.turret_motor.setTargetPosition(NINETY_DEGREES_LEFT);
                        hardwarePowerPlay.turret_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.turret_motor.setPower(1);
                        highGoalLeft = HighGoalLeft.LIFT_TRANSITION_1;
                    }
                    break;
                case LIFT_TRANSITION_1:
                    if (Math.abs(hardwarePowerPlay.turret_motor.getCurrentPosition() - LIFT_HIGH) < 10) {
                        liftPosition = liftWait;
                        highGoalLeft = HighGoalLeft.LIFT_UP_1;
                    }
                    break;
                default:
                    highGoalLeft = HighGoalLeft.LIFT_UP_1;
            }

            if (dPadLeftPressed) {
                if (dPadLeftPressed == false) {
                    dPadLeftPressed = true;
                    if (liftPosition == liftOff) {
                        liftPosition = liftUp2;
                    }
                    else if (liftPosition == liftWait) {
                        liftPosition = liftDown;
                    }
                }
            }
            else dPadLeftPressed = false;

            switch (midGoalLeft) {
                case LIFT_UP_2:
                    if (liftPosition == liftUp2) {
                        hardwarePowerPlay.lift_motor_1.setTargetPosition(LIFT_MID);
                        hardwarePowerPlay.lift_motor_2.setTargetPosition(LIFT_MID);
                        hardwarePowerPlay.lift_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_1.setPower(1);
                        hardwarePowerPlay.lift_motor_2.setPower(1);
                        midGoalLeft = MidGoalLeft.LIFT_TURN_2;
                    }
                    break;
                case LIFT_TURN_2:
                    if (Math.abs(hardwarePowerPlay.lift_motor_1.getCurrentPosition() - LIFT_HIGH) < 10) {
                        hardwarePowerPlay.extension_servo.setPosition(EXTEND_POS);
                        hardwarePowerPlay.turret_motor.setTargetPosition(NINETY_DEGREES_LEFT);
                        hardwarePowerPlay.turret_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.turret_motor.setPower(1);
                        midGoalLeft= MidGoalLeft.LIFT_TRANSITION_2;
                    }
                    break;
                case LIFT_TRANSITION_2:
                    if (Math.abs(hardwarePowerPlay.turret_motor.getCurrentPosition() - LIFT_HIGH) < 10) {
                        liftPosition = liftWait;
                        midGoalLeft = MidGoalLeft.LIFT_UP_2;
                    }
                    break;
                default:
                    midGoalLeft = MidGoalLeft.LIFT_UP_2;
            }

            if (dPadRightPressed) {
                if (dPadRightPressed == false) {
                    dPadRightPressed = true;
                    if (liftPosition == liftOff) {
                        liftPosition = liftUp3;
                    }
                    else if (liftPosition == liftWait) {
                        liftPosition = liftDown;
                    }
                }
            }
            else dPadRightPressed = false;

            switch (lowGoalLeft) {
                case LIFT_UP_3:
                    if (liftPosition == liftUp1) {
                        hardwarePowerPlay.lift_motor_1.setTargetPosition(LIFT_LOW);
                        hardwarePowerPlay.lift_motor_2.setTargetPosition(LIFT_LOW);
                        hardwarePowerPlay.lift_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_1.setPower(1);
                        hardwarePowerPlay.lift_motor_2.setPower(1);
                        lowGoalLeft = LowGoalLeft.LIFT_TURN_3;
                    }
                    break;
                case LIFT_TURN_3:
                    if (Math.abs(hardwarePowerPlay.lift_motor_1.getCurrentPosition() - LIFT_HIGH) < 10) {
                        hardwarePowerPlay.extension_servo.setPosition(EXTEND_POS);
                        hardwarePowerPlay.turret_motor.setTargetPosition(NINETY_DEGREES_LEFT);
                        hardwarePowerPlay.turret_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.turret_motor.setPower(1);
                        lowGoalLeft = LowGoalLeft.LIFT_TRANSITION_3;
                    }
                    break;
                case LIFT_TRANSITION_3:
                    if (Math.abs(hardwarePowerPlay.turret_motor.getCurrentPosition() - LIFT_HIGH) < 10) {
                        liftPosition = liftWait;
                        lowGoalLeft = LowGoalLeft.LIFT_UP_3;
                    }
                    break;
                default:
                    lowGoalLeft = LowGoalLeft.LIFT_UP_3;
            }

            if (dPadDownPressed) {
                if (dPadDownPressed == false) {
                    dPadDownPressed = true;
                    if (liftPosition == liftOff) {
                        liftPosition = liftUp4;
                    }
                    else if (liftPosition == liftWait) {
                        liftPosition = liftDown2;
                    }
                }
            }
            else dPadDownPressed = false;

            switch (groundGoalLeft) {
                case LIFT_UP_4:
                    if (liftPosition == liftUp1) {
                        hardwarePowerPlay.extension_servo.setPosition(EXTEND_POS);
                        hardwarePowerPlay.lift_motor_1.setTargetPosition(LIFT_GROUND);
                        hardwarePowerPlay.lift_motor_2.setTargetPosition(LIFT_GROUND);
                        hardwarePowerPlay.lift_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_1.setPower(1);
                        hardwarePowerPlay.lift_motor_2.setPower(1);
                        groundGoalLeft = GroundGoalLeft.LIFT_TURN_4;
                    }
                    break;
                case LIFT_TURN_4:
                    if (Math.abs(hardwarePowerPlay.lift_motor_1.getCurrentPosition() - LIFT_HIGH) < 10) {
                        hardwarePowerPlay.turret_motor.setTargetPosition(NINETY_DEGREES_LEFT);
                        hardwarePowerPlay.turret_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.turret_motor.setPower(1);
                        groundGoalLeft = GroundGoalLeft.LIFT_TRANSITION_4;
                    }
                    break;
                case LIFT_TRANSITION_4:
                    if (Math.abs(hardwarePowerPlay.turret_motor.getCurrentPosition() - LIFT_HIGH) < 10) {
                        liftPosition = liftWait;
                        groundGoalLeft = GroundGoalLeft.LIFT_UP_4;
                    }
                    break;
                default:
                    groundGoalLeft = GroundGoalLeft.LIFT_UP_4;
            }

            switch (retractLift) {
                case LIFT_SCORE:
                    if (liftPosition == liftDown) {
                        hardwarePowerPlay.grip_servo.setPosition(OPEN_GRIP);
                        scoreTimer.reset();
                        retractLift = RetractLift.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if (scoreTimer.seconds() >= SCORE_TIME) {
                        hardwarePowerPlay.extension_servo.setPosition(INTAKE_POS);
                        hardwarePowerPlay.turret_motor.setTargetPosition(INTAKE_ROT);
                        hardwarePowerPlay.turret_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.turret_motor.setPower(1);
                        retractLift = RetractLift.LIFT_DOWN;
                    }
                    break;
                case LIFT_DOWN:
                    if ((Math.abs(hardwarePowerPlay.turret_motor.getCurrentPosition()) - INTAKE_ROT) < 10) {
                        hardwarePowerPlay.lift_motor_1.setTargetPosition(LIFT_INTAKE);
                        hardwarePowerPlay.lift_motor_2.setTargetPosition(LIFT_INTAKE);
                        hardwarePowerPlay.lift_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_1.setPower(1);
                        hardwarePowerPlay.lift_motor_2.setPower(1);
                        hardwarePowerPlay.grip_servo.setPosition(OPEN_GRIP);
                        retractLift = RetractLift.LIFT_RESET;
                    }
                    break;
                case LIFT_RESET:
                    if ((Math.abs(hardwarePowerPlay.lift_motor_1.getCurrentPosition()) - LIFT_INTAKE) < 10) {
                        liftPosition = liftOff;
                        retractLift = RetractLift.LIFT_SCORE;
                    }
                    break;
                default:
                    retractLift = RetractLift.LIFT_SCORE;
            }

            switch (retractLift2) {
                case LIFT_SCORE_2:
                    if (liftPosition == liftDown) {
                        hardwarePowerPlay.grip_servo.setPosition(OPEN_GRIP);
                        scoreTimer.reset();
                        retractLift2 = RetractLift2.LIFT_RETRACT_2;
                    }
                    break;
                case LIFT_PARTIAL_UP_2:
                    if (scoreTimer.seconds() >= SCORE_TIME) {
                        hardwarePowerPlay.lift_motor_1.setTargetPosition(LIFT_PARTIAL);
                        hardwarePowerPlay.lift_motor_2.setTargetPosition(LIFT_PARTIAL);
                        hardwarePowerPlay.lift_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_1.setPower(1);
                        hardwarePowerPlay.lift_motor_2.setPower(1);
                        retractLift2 = RetractLift2.LIFT_RETRACT_2;
                    }
                case LIFT_RETRACT_2:
                    if ((Math.abs(hardwarePowerPlay.lift_motor_1.getCurrentPosition()) - LIFT_PARTIAL) < 10) {
                        hardwarePowerPlay.extension_servo.setPosition(INTAKE_POS);
                        hardwarePowerPlay.turret_motor.setTargetPosition(INTAKE_ROT);
                        hardwarePowerPlay.turret_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.turret_motor.setPower(1);
                        retractLift2 = RetractLift2.LIFT_DOWN_2;
                    }
                    break;
                case LIFT_DOWN_2:
                    if ((Math.abs(hardwarePowerPlay.turret_motor.getCurrentPosition()) - INTAKE_ROT) < 10) {
                        hardwarePowerPlay.lift_motor_1.setTargetPosition(LIFT_INTAKE);
                        hardwarePowerPlay.lift_motor_2.setTargetPosition(LIFT_INTAKE);
                        hardwarePowerPlay.lift_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwarePowerPlay.lift_motor_1.setPower(1);
                        hardwarePowerPlay.lift_motor_2.setPower(1);
                        hardwarePowerPlay.grip_servo.setPosition(OPEN_GRIP);
                        retractLift2 = RetractLift2.LIFT_RESET_2;
                    }
                    break;
                case LIFT_RESET_2:
                    if ((Math.abs(hardwarePowerPlay.lift_motor_1.getCurrentPosition()) - LIFT_INTAKE) < 10) {
                        liftPosition = liftOff;
                        retractLift2 = RetractLift2.LIFT_SCORE_2;
                    }
                    break;
                default:
                    retractLift2 = RetractLift2.LIFT_SCORE_2;
            }


        }
    }
}
