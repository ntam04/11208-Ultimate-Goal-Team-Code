package org.firstinspires.ftc.teamcode.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Freight Frenzy TeleOp Blue")
public class FreightFrenzyTeleOpBlue extends LinearOpMode {

    HardwareFreightFrenzy hardwareFreightFrenzy = new HardwareFreightFrenzy();

    //Stage 3
    public enum LiftState {
        LIFT_START,
        LIFT_EXTEND,
        LIFT_DUMP,
        LIFT_RETRACT
    }

    //Stage 2
    public enum LiftState_2 {
        LIFT_START_2,
        LIFT_EXTEND_2,
        LIFT_DUMP_2,
        LIFT_RETRACT_2
    }

    LiftState liftState = LiftState.LIFT_START;
    LiftState_2 liftState2 = LiftState_2.LIFT_START_2;

    @Override
    public void runOpMode() throws InterruptedException {

        hardwareFreightFrenzy.init(hardwareMap);

        hardwareFreightFrenzy.outtake_servo.setPosition(0.3);

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

        final double DUMP_IDLE;
        final double DUMP_DEPOSIT;
        final double DUMP_TIME;
        final int LIFT_LOW;
        final int LIFT_HIGH;
        final int LIFT_HIGH_2;
        DUMP_IDLE = 0.3;
        DUMP_DEPOSIT = 0.7;
        DUMP_TIME = 0.5;
        LIFT_LOW = 0;
        LIFT_HIGH = 1400;
        LIFT_HIGH_2 = 1000;
        ElapsedTime liftTimer = new ElapsedTime();

        final double INTAKE_DOWN;
        final double INTAKE_UP;
        final double INTAKE_MID;
        final int HANDOFF_POS;
        INTAKE_DOWN = 1;
        INTAKE_UP = 0.28;
        INTAKE_MID = 0.5;
        HANDOFF_POS = 0;

        boolean xPressed;
        double wheelOn;
        double wheelOff;
        double wheelMode;
        double wheelVelocity;
        xPressed = false;
        wheelOn = 1;
        wheelOff = 2;
        wheelMode = wheelOff;
        wheelVelocity = -1500;

        boolean rightBumperPressed;
        boolean rightTriggerPressed;
        double intakeIn;
        double intakeOff;
        double intakeOut;
        double intakeMode;
        rightBumperPressed = false;
        rightTriggerPressed = false;
        intakeIn = 1;
        intakeOff = 2;
        intakeOut = 3;
        intakeMode = intakeOff;


        liftTimer.reset();


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
                    } else if (motorPower == halfPower) {
                        motorPower = fullPower;
                    }
                }
            } else {
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
            hardwareFreightFrenzy.front_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);
            hardwareFreightFrenzy.back_right_motor.setPower(Range.clip
                    (gamepad1.right_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);
            hardwareFreightFrenzy.front_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y - (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);
            hardwareFreightFrenzy.back_left_motor.setPower(Range.clip
                    (gamepad1.left_stick_y + (gamepad1.left_stick_x + gamepad1.right_stick_x), -1, 1) * motorPowerInput);

            //Intake
            hardwareFreightFrenzy.extension_motor_1.setPower(gamepad2.right_stick_y);
            hardwareFreightFrenzy.extension_motor_2.setPower(gamepad2.right_stick_y);

            if (gamepad2.dpad_down) {
                hardwareFreightFrenzy.drop_servo_1.setPosition(INTAKE_DOWN);
                hardwareFreightFrenzy.drop_servo_2.setPosition(INTAKE_DOWN);
                intakeMode = intakeIn;
            }
            else if (gamepad2.dpad_up) {
                hardwareFreightFrenzy.drop_servo_1.setPosition(INTAKE_UP);
                hardwareFreightFrenzy.drop_servo_2.setPosition(INTAKE_UP);
            }
            else if (gamepad2.dpad_right) {
                hardwareFreightFrenzy.drop_servo_1.setPosition(INTAKE_MID);
                hardwareFreightFrenzy.drop_servo_2.setPosition(INTAKE_MID);
            }

            /*
            Turns on intake if right bumper is pressed
            Turns off intake if left or right triggers are held
             */
            if (gamepad2.right_bumper) {
                if (rightBumperPressed == false) {
                    rightBumperPressed = true;
                    if (intakeMode == intakeOff) {
                        intakeMode = intakeIn;
                    }
                    else if (intakeMode == intakeIn) {
                        intakeMode = intakeOff;
                    }
                    else if (intakeMode == intakeOut) {
                        intakeMode = intakeIn;
                    }
                }
            }
            else if (gamepad2.right_trigger >= 0.25) {
                if (rightTriggerPressed == false) {
                    rightTriggerPressed = true;
                    if (intakeMode == intakeOff) {
                        intakeMode = intakeOut;
                    }
                    else if (intakeMode == intakeIn) {
                        intakeMode = intakeOut;
                    }
                    else if (intakeMode == intakeOut) {
                        intakeMode = intakeOut;
                    }
                }
            }
            else rightBumperPressed = false; rightTriggerPressed = false;

            /*
            Turns intake on when right bumper is pressed
            Turns intake off when right bumper is pressed again

            Reverses intake when left or right triggers are held
             */
            if (intakeMode == intakeIn) {
                hardwareFreightFrenzy.intake_servo.setPower(1);
                telemetry.addData("Intake Mode: ", "On");
            }
            else if (intakeMode == intakeOut) {
                if ((gamepad2.right_trigger >= 0.25)) {
                    hardwareFreightFrenzy.intake_servo.setPower(-1);
                }
                else hardwareFreightFrenzy.intake_servo.setPower(0);

                telemetry.addData("Intake Mode: ", "Out");
            }
            else if (intakeMode == intakeOff) {
                hardwareFreightFrenzy.intake_servo.setPower(0);
                telemetry.addData("Intake Mode: ", "Off");
            }

            //Outtake
            //Stage 3 automation
            switch (liftState) {
                case LIFT_START:
                    if (gamepad1.dpad_up) {
                        //if dpad_up is pressed, extend lift to Stage 2
                        hardwareFreightFrenzy.lift_motor.setTargetPosition(LIFT_HIGH);
                        hardwareFreightFrenzy.lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwareFreightFrenzy.lift_motor.setPower(1);
                        liftState = LiftState.LIFT_EXTEND;
                    }
                    break;
                case LIFT_EXTEND:
                    if (Math.abs(hardwareFreightFrenzy.lift_motor.getCurrentPosition() - LIFT_HIGH) < 10) {
                        hardwareFreightFrenzy.outtake_servo.setPosition(DUMP_DEPOSIT);

                        liftTimer.reset();
                        liftState = LiftState.LIFT_DUMP;
                    }
                    break;
                case LIFT_DUMP:
                    if (liftTimer.seconds() >= DUMP_TIME) {
                        hardwareFreightFrenzy.outtake_servo.setPosition(DUMP_IDLE);
                        hardwareFreightFrenzy.lift_motor.setTargetPosition(LIFT_LOW);
                        hardwareFreightFrenzy.lift_motor.setPower(1);
                        liftState = LiftState.LIFT_RETRACT;
                    }
                    break;
                case LIFT_RETRACT:
                    if (Math.abs(hardwareFreightFrenzy.lift_motor.getCurrentPosition() - LIFT_LOW) < 10) {
                        liftState = LiftState.LIFT_START;
                    }
                    break;
                default:
                    liftState = LiftState.LIFT_START;
            }

            //Stage 2 automation
            switch (liftState2) {
                case LIFT_START_2:
                    if (gamepad1.dpad_right) {
                        //if dpad_right is pressed, extend lift to Stage 3
                        hardwareFreightFrenzy.lift_motor.setTargetPosition(LIFT_HIGH_2);
                        hardwareFreightFrenzy.lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        hardwareFreightFrenzy.lift_motor.setPower(1);
                        liftState2 = LiftState_2.LIFT_EXTEND_2;
                    }
                    break;
                case LIFT_EXTEND_2:
                    if (Math.abs(hardwareFreightFrenzy.lift_motor.getCurrentPosition() - LIFT_HIGH_2) < 10) {
                        hardwareFreightFrenzy.outtake_servo.setPosition(DUMP_DEPOSIT);

                        liftTimer.reset();
                        liftState2 = LiftState_2.LIFT_DUMP_2;
                    }
                    break;
                case LIFT_DUMP_2:
                    if (liftTimer.seconds() >= DUMP_TIME) {
                        hardwareFreightFrenzy.outtake_servo.setPosition(DUMP_IDLE);
                        hardwareFreightFrenzy.lift_motor.setTargetPosition(LIFT_LOW);
                        hardwareFreightFrenzy.lift_motor.setPower(1);
                        liftState2 = LiftState_2.LIFT_RETRACT_2;
                    }
                    break;
                case LIFT_RETRACT_2:
                    if (Math.abs(hardwareFreightFrenzy.lift_motor.getCurrentPosition() - LIFT_LOW) < 10) {
                        liftState2 = LiftState_2.LIFT_START_2;
                    }
                    break;
                default:
                    liftState2 = LiftState_2.LIFT_START_2;
            }

            if (gamepad2.left_trigger >= 0.1) {
                hardwareFreightFrenzy.duck_wheel.setVelocity(wheelVelocity);
            }
            else {
                hardwareFreightFrenzy.duck_wheel.setVelocity(0);
            }
        }
    }
}
