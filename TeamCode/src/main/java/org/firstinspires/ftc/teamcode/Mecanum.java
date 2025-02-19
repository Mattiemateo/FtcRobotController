package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Mecanum extends LinearOpMode {
    private DcMotor arm;
    private DcMotor wrist;
    private Servo claw;
    private Servo intake;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    int armpos = 0;
    int targetpos = 0;
    boolean is_open_claw = true;
    boolean is_open_intake = false;
    float timer_intake = 0;

    @Override
    public void runOpMode() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");  // FIXED
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive"); // FIXED
        arm = hardwareMap.get(DcMotor.class, "arm");
        intake = hardwareMap.get(Servo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(DcMotor.class, "wrist");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Waiting for input");
            telemetry.addData("lx", gamepad1.left_stick_x);
            telemetry.addData("ly", gamepad1.left_stick_y);
            telemetry.addData("rx", gamepad1.right_stick_x);

            telemetry.addData("Arm-pos", arm.getCurrentPosition());
            telemetry.addData("Wrist-pos", wrist.getCurrentPosition());
            telemetry.addData("is-open_claw", is_open_claw);
            telemetry.addData("is-open_intake", is_open_intake);
            telemetry.addData("timer_intake", timer_intake);

            telemetry.update();

            armpos = arm.getCurrentPosition();

            if (gamepad1.left_trigger < 0.5) {
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);

                double leftFront = power * cos + turn;
                double rightFront = power * sin - turn;
                double leftBack = power * sin + turn;
                double rightBack = power * cos - turn;

                if ((power + Math.abs(turn)) > 1) {
                    leftFront /= power + turn;
                    leftBack /= power + turn;
                    rightFront /= power + turn;
                    rightBack /= power + turn;
                }

                leftFrontDrive.setPower(leftFront);
                leftBackDrive.setPower(leftBack);
                rightFrontDrive.setPower(rightFront);
                rightBackDrive.setPower(rightBack);

            } else if (gamepad1.left_trigger > 0.5) {
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;

                double theta = Math.atan2(y, x);
                double power = Math.hypot(x, y);

                double sin = Math.sin(theta - Math.PI / 4);
                double cos = Math.cos(theta - Math.PI / 4);

                double leftFront = power * cos + turn;
                double rightFront = power * sin - turn;
                double leftBack = power * sin + turn;
                double rightBack = power * cos - turn;

                if ((power + Math.abs(turn)) > 1) {
                    leftFront /= power + turn;
                    leftBack /= power + turn;
                    rightFront /= power + turn;
                    rightBack /= power + turn;
                }

                leftFrontDrive.setPower(leftFront * 0.25);
                leftBackDrive.setPower(leftBack * 0.25);
                rightFrontDrive.setPower(rightFront * 0.25);
                rightBackDrive.setPower(rightBack * 0.25);
            }


            if (gamepad2.square) {
                if (is_open_claw){
                    claw.setPosition(0.6);
                    is_open_claw = false;
                }
            }else if(gamepad2.cross) {
                if (!is_open_claw){
                    claw.setPosition(0.4);
                    is_open_claw = true;
                }
            }

            if (gamepad2.triangle) {
                if (is_open_intake) {
                    gamepad1.rumble(500);
                    intake.setPosition(1);
                    is_open_intake = false;
                }
            } else if(gamepad2.circle) {
                if (!is_open_intake) {
                    intake.setPosition(0);
                    is_open_intake = true;
                }
            }

            //if (timer_intake > 0) {
            //    timer_intake -= 0.01F;
            //}

            if (gamepad2.dpad_up) {
                arm.setPower(0.6);
                targetpos = 0;
            } else if (gamepad2.dpad_down) {
                if (armpos > 200) {
                    arm.setPower(-0.6);
                }
                targetpos = 0;
            } else {
                if (targetpos == 0) {
                    targetpos = armpos;
                }
                if (targetpos + 15 < armpos) {
                    arm.setPower(-0.1);
                } else if (targetpos - 15 > armpos){
                    arm.setPower(0.1);
                } else {
                    arm.setPower(0);
                }
            }


            if (gamepad2.dpad_left) {
                wrist.setPower(0.6);
            } else if (gamepad2.dpad_right) {
                wrist.setPower(-0.6);
            } else {
                wrist.setPower(0);
            }


        }

    }
}
