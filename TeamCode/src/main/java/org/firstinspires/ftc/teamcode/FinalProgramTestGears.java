package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

@TeleOp(name = "Final(AYY LMAO PICK ME PICK ME" +
        "")
public class FinalProgramTestGears extends LinearOpMode {

    LZRobot robot = new LZRobot();
    //LZNavSys nav = new LZNavSys();

    public void runOpMode() {

        robot.init(hardwareMap, this);
       // nav.initVuforia(this, robot);

        robot.s5.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

      //  nav.activateTracking();

        while (opModeIsActive()) {

         //   telemetry.addData("Relic:", nav.whatRelic());

            if(gamepad2.left_stick_y != 0) {

                robot.s6.setPosition(abs(gamepad2.left_stick_y - 1));
                sleep(50);
                robot.s5.setPosition(abs(gamepad2.left_stick_y - 1));

            } else {

                robot.s6.setPosition(1);
                sleep(50);
                robot.s5.setPosition(1);

            }

            robot.slide.setPower(gamepad2.right_stick_y * 0.5);

            robot.manualDrive();

            robot.moveRobot();

            robot.dpadDrive(0.75);

            telemetry.addData("Blue", robot.color.blue());
            telemetry.addData("Red", robot.color.red());
            telemetry.addData("Range", robot.range1.getDistance(DistanceUnit.CM));
            telemetry.addData("ServoL", robot.s5.getPosition());
            telemetry.addData("ServoR", robot.s6.getPosition());

            telemetry.update();

        }
    }
}