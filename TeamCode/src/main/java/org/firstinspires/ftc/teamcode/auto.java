package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.ArmController;

@Autonomous
public class auto extends LinearOpMode {
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
    boolean is_open = true;

    private void move_left(int mm) {
        leftFrontDrive.setPower(-0.5);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightBackDrive.setPower(-0.5);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void move_right(int mm) {
        leftFrontDrive.setPower(0.5);
        rightFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightBackDrive.setPower(0.5);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void move_y(int mm, float forward) {
        leftFrontDrive.setPower(0.5 * forward);
        rightFrontDrive.setPower(0.5 * forward);
        leftBackDrive.setPower(0.5 * forward);
        rightBackDrive.setPower(0.5 * forward);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void turn(int mm, float left) {
        leftFrontDrive.setPower(0.25 * left);
        rightFrontDrive.setPower(-0.25 * left);
        leftBackDrive.setPower(0.25 * left);
        rightBackDrive.setPower(-0.25 * left);
        sleep(mm);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void arm(int up) {
        if (up == 1) {
            arm.setPower(0.6);
            targetpos = 0;
        } else if (up == -1) {
            arm.setPower(-0.6);
            targetpos = 0;
        } else {
            if (targetpos == 0) {
                targetpos = armpos;
            }
            if (targetpos + 15 < armpos) {
                arm.setPower(-0.1);
            } else if (targetpos - 15 > armpos) {
                arm.setPower(0.1);
            } else {
                arm.setPower(0);
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
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

        // run until the end of the match (driver presses STOP)
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

            // Start the arm movement in a separate thread
            Thread armThread = new Thread(() -> {
                arm(1);
                sleep(1000);
                arm(0);
                while (true){
                    arm(0);
                }
            });

            armThread.start();

            sleep(1500);
            move_y(1000, 1);
            sleep(1000);
            move_right(1000);
            sleep(250);
            move_y(1000, -1);
            armThread.join();
            sleep(30000 - 1000 + 250 + 500 + 500);

            // Make sure the arm finishes before ending the OpMode.


        }

    }
}
