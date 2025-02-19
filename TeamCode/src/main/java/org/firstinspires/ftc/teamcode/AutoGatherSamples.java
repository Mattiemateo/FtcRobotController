package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

@Autonomous
public class AutoGatherSamples extends LinearOpMode {
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

        if (opModeIsActive()) {
            // hold tight and make sure we don't trip...
            closeClaw();
            moveArmToPosition(500);

            // Start the arm movement in a separate thread
            Thread hookFirstThread = new Thread(() -> {
                closeClaw();
                moveArmToPosition(1000); // lift...
                sleep(2000);
                moveArmToPosition(2900); // Suspend high above the bar...
                sleep(3000); // aaaaand...
                moveArmToPosition(2200, 1.0f); // ..BAM!! Hook the specimen!
                sleep(1000);
                openClaw();
                sleep(4000);
                moveArmToPosition(1500);
            });

            Thread bringFirstSpecimenThread = new Thread(() -> {
                // Drive forward while arm moves
                sleep(1000);
                move_y(900 , 1F, 0.4f);
                sleep(1000);
                move_left(1500);
                turn(200, -1);
                sleep(1000);
                move_y(650 , 1F, 0.3f); //
                // HOOKING... Wait until claw's open again
                sleep(3000);
                // Let's move back for another specimen!
                move_y(950, -1F, 0.3f); // back up
                sleep(500);
                //turn(3800, 1); // rotate so we can pick up a new specimen...
                move_right(2000); // align... (this needs to be more to reach landing zone)
                turn(300, 1);
                sleep(500);
                move_y(1500, 1F, 0.5f); // approach new specimen
                sleep(500);
                move_right(700);
                move_y(1900, -1F, 0.6f);
                move_y(1900, 1F, 0.6f);
                move_right(700);
                move_y(1900, -1F, 0.6f);
                turn(50, -1);
                move_y(1900, 1F, 0.6f);
                move_right(850);
                move_y(1900, -1F, 0.6f);
            });

            hookFirstThread.start();
            bringFirstSpecimenThread.start();

            sleep(30000);

            // Make sure the arm finishes before ending the opmode
            hookFirstThread.join();
            bringFirstSpecimenThread.join();
        }
    }

    private void openClaw() {
        if (!is_open_claw){
            claw.setPosition(0.4);
            is_open_claw = true;
        }
    }

    private void closeClaw() {
        if (is_open_claw){
            claw.setPosition(0.6);
            is_open_claw = false;
        }
    }

    private void moveArmToPosition(int pos) {
        moveArmToPosition(pos, 0.6f);
    }

    private void moveArmToPosition(int pos, float atSpeed) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(atSpeed);
        while (arm.isBusy()) {
            sleep(30);
        }
    }

    private void holdArmStill() {
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0.0f);
    }

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

    private void move_y(int ms, float forward, float speed) {
        leftFrontDrive.setPower(speed * forward);
        rightFrontDrive.setPower(speed * forward);
        leftBackDrive.setPower(speed * forward);
        rightBackDrive.setPower(speed * forward);
        sleep(ms);
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
}
