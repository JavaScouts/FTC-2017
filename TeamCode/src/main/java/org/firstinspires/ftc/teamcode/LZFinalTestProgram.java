package org.firstinspires.ftc.teamcode;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp(name = "Leon's Final Program")
public class LZFinalTestProgram extends LinearOpMode {

    LZTestChassis robot = new LZTestChassis();
    float X1=0;
    float X2=0;
    float Y1=0;
    double threshold = 0.05;

    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            //float s1 = gamepad2.left_stick_y;
            //float s2 = gamepad2.right_stick_y;
            if(gamepad2.x) {
                robot.servo1.setPosition(0.64);
                robot.servo2.setPosition(0.34);
            } else {
                robot.servo1.setPosition(0);
                robot.servo2.setPosition(0.75);
            }

            telemetry.addData("servopos", robot.servo1.getPosition());
            telemetry.addData("servo2pos", robot.servo2.getPosition());
            telemetry.update();

            float leftY = gamepad1.left_stick_y;
            float leftX = gamepad1.left_stick_x;
            float rightX = gamepad1.right_stick_x;

            if(abs(leftY) > threshold) {
                Y1=leftY;
            } else {
                Y1=0;
            }
            if(abs(leftX) > threshold) {
                X1=leftX;
            } else {
                X1=0;
            }
            if(abs(rightX) > threshold) {
                X2=rightX;
            } else {
                X2=0;
            }
            robot.FrontMotor1.setPower(Y1 - X2 - X1);
            robot.BackMotor1.setPower(Y1 - X2 + X1);
            robot.FrontMotor2.setPower(Y1 + X2 + X1);
            robot.BackMotor2.setPower(Y1 + X2 - X1);
            }
        }
    }