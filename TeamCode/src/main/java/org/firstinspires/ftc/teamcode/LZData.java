package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;;

/**
 * Created by Liam on 11/5/2017.
 */
@TeleOp(name = "LZDATA")
public class LZData extends LinearOpMode {

    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor slide;

    public void runOpMode() {

        FrontMotor1 = hardwareMap.dcMotor.get("fl");
        FrontMotor2 = hardwareMap.dcMotor.get("fr");
        slide = hardwareMap.dcMotor.get("Arm");
        BackMotor1 = hardwareMap.dcMotor.get("bl");
        BackMotor2 = hardwareMap.dcMotor.get("br");

        BackMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()) {

            float lF = -gamepad1.left_stick_y;
            float rF = -gamepad1.right_stick_y;
            float lB = -gamepad2.left_stick_y;
            float rB = -gamepad2.right_stick_y;

            FrontMotor1.setPower(lF);
            FrontMotor2.setPower(rF);
            BackMotor1.setPower(lB);
            BackMotor2.setPower(rB);

            /////slidePos = slide.getCurrentPosition();

            telemetry.addData("Gamepad Powers", "LF[%+5.2f], RF[%+5.2f], LB[%+5.2f]", lF, rF, lB);
            //telemetry.addData("ArmClicks", slidePos);
            telemetry.update();

        }


    }

}
