/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Kaleb VueMark Test", group ="Concept")
@Disabled
public class VuMarkKalebTest extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


        parameters.vuforiaLicenseKey = "Acbhfjv/////AAAAGQwVGIAmv0V3nNH7nrtPGJVSuirk278Uo+j+394hRfZGLNmucewzhlA4ux8ZUQz0OpL1mJuPW+lKbjippfmpGiiXsvpnm2p6prlsNHzZNthjThZyFArOkXDkjL5bYlVT3zlo3oc+XNg/W4rBdXHeBpw6OgO1a7D0xhGHkqaihUWqeESvWtcH+uSJXha/umtRGu0DIPRW0n4a3Z0cjbNzhicHvrGOWuwuVhyua1IcrOqed3/bTdts+JhuG3TakwFBb1GpIkWXNziiorUUXVstPVNVrty3AnesNZ11gsJyHVu3YCKi//itjkqr/6xmINr4LDrwdf1Pg2e9L9iT9qtFWjrrnEQYGzxgpnrj0+PvzWWw";


        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.cameraMonitorFeedback= VuforiaLocalizer.Parameters.CameraMonitorFeedback.BUILDINGS;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackable = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        //VuforiaTrackable[] relicTemplate = {relicTrackables.get(0), relicTrackables.get(1), relicTrackables.get(2)};
        //relicTemplate[0].setName("0"); // can help in debugging; otherwise not necessary
        //relicTemplate[1].setName("1");
        //relicTemplate[2].setName("2");

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackable.activate();
        while (opModeIsActive()) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTrackable.get(0));
            if (vuMark == RelicRecoveryVuMark.CENTER) {
                telemetry.addData("VueMark", "Center Visible");
            }else{

                telemetry.addData("VueMark","Center not Visible");
            }
            if (vuMark == RelicRecoveryVuMark.LEFT) {
                telemetry.addLine();
                telemetry.addData("VueMark", "Left Visible");
            }else{
                telemetry.addLine();
                telemetry.addData("VueMark","Left not Visible");
            }
            if (vuMark == RelicRecoveryVuMark.RIGHT) {
                telemetry.addLine();
                telemetry.addData("VueMark", "Right Visible");
            }else{
                telemetry.addLine();
                telemetry.addData("VueMark","Right not Visible");
            }
            telemetry.update();
        }

    }
}
