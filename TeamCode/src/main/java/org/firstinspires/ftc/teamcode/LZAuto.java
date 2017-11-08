package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Liam on 10/26/2017.
 */

public class LZAuto extends LinearOpMode {

    LZRobot robot = new LZRobot();
    VuforiaLocalizer vuforia;

    public void runOpMode() {

        robot.init(hardwareMap, this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "Abd1Glv/////AAAAGZmYidRnmEU+hDYHKdO9PSlKOkMzut3jACYyynIBk/aI/uy1g5waDFv0hQlDLLoROL/RcHOLRIXYoEeTj0xW6JPELJd94fYr72YQ8A/hFOPLDO1UuM1je+Y2KbABDDilKaqShhHzfPinH1M7NLA7aZCwUk4ZRiDsLcB9f4hKVa9g//sUxId3KFb4GW438tD8t/xdZfcLcm+vP4yREaW6NirbqzCwTvp22/2eDuKIHGvVn3Fju4PcqCYYaFTvubLX2iXOwrEJWEVD+qtvQsqr5Dk+ClaUlv9amad/14aEU5jUohRU04PUlEgtaGSfLGNBmOXW14ugnipSbC1Lr423HlaaGgNjseX+D3nbZlX9a2Zo";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        waitForStart();

        relicTrackables.activate();

        while(opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if(vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("Visible:", vuMark); //to get telemetry data

            }

            /* pretend code
            *
            *
            *
            *
            *
            *
            *
            *
            *
            * */

            if(vuMark == RelicRecoveryVuMark.CENTER)    {

                /* blah blah blah
                *
                *
                *
                */


            }

        }

    }

}
