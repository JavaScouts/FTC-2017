package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

@TeleOp(name = "NewDriveSys")
public class NEWDRIVESYSTEM extends LinearOpMode {

    LZRobot robot = new LZRobot();
    LZNavSys nav = new LZNavSys();
    DcMotor Arm;
    Servo s1;
    final double TARGET_DISTANCE = 400.0;


    public void runOpMode() {

        robot.init(hardwareMap, this);
        nav.initVuforia(this, robot);
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

            if(gamepad1.x) {
                robot.move("left", 1);
            }   else if(gamepad1.b) {
                robot.move("right", 1);
            }



            if (nav.targetsAreVisible() && gamepad1.left_bumper) {
                // Calculate automatic target approach
                nav.cruiseControl(TARGET_DISTANCE);

            } else {
                // Drive the robot using the joysticks
                robot.manualDrive();
            }

            // Build telemetry messages with Navigation Information;
            nav.addNavTelemetry();




            robot.manualDrive();

            robot.moveRobot();

            telemetry.update();
        }
    }
}