package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

@TeleOp(name = "NewDriveSys")
public class NEWDRIVESYSTEM extends LinearOpMode {

    LZRobot robot = new LZRobot();
    DcMotor Arm;
    Servo s1;

    public void runOpMode() {

        robot.init(hardwareMap, this);
        Arm = hardwareMap.dcMotor.get("Arm");
        s1 = hardwareMap.servo.get("s1");

        telemetry.addData("Initialization", "Completed");
        telemetry.update();
        s1.setPosition(0);

        waitForStart();

        while (opModeIsActive()) {

            float left90 = -gamepad2.left_stick_y;

            Arm.setPower(left90);

            if (gamepad2.a) {
                s1.setPosition(1.0);
            } else if (gamepad2.b) {
                s1.setPosition(0);
            }




            robot.manualDrive();

            robot.moveRobot();

            telemetry.update();
        }
    }
}