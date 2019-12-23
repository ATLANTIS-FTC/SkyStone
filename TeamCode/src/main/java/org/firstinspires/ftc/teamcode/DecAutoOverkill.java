/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.android.dex.EncodedValueCodec;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "DecAutoOverkill", group = "December")
//@Disabled
public class DecAutoOverkill extends LinearOpMode {

    OctHardware robot   = new OctHardware();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public String pitch;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    int ssX = -1;
    int sOneX = -1;
    int sTwoX = -1;
//
//    Stone stoneOne = new Stone();
//    Stone stoneTwo = new Stone();
//    Stone stoneThree = new Stone();
//    Stone stoneFour = new Stone();
//    Stone stoneFive = new Stone();
//    Stone stoneSix = new Stone();

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.

        robot.init(hardwareMap);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking - U R GO, GOOD LUCK!");
        telemetry.update();
        waitForStart();

        boolean found = false;
        int key = -1;

//        robot.intakeFlipperLeft.setPosition(.65);
//        robot.intakeFlipperRight.setPosition(.980);
//        encoderAccessory(1,300);
//        robot.intakeFlipperLeft.setPosition(.5);
//        robot.intakeFlipperRight.setPosition(.8);
        robot.openerRight.setPosition(.2);
        robot.openerLeft.setPosition(.15);
        encoderAccessory(1,40);
        encoderDrive(1,0,1000,1);

        runtime.reset(); //TensorFlow Timer Wait
        while (opModeIsActive() && runtime.seconds() < 1.75) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    if (updatedRecognitions.size() == 3) { //if 3 minerals detected, use TF mandated algorithm
                        int skystoneX = -1;
                        int stoneOneX = -1;
                        int stoneTwoX = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals("Skystone")) {
                                skystoneX = (int) recognition.getLeft();
                            } else if (stoneOneX == -1) {
                                stoneOneX = (int) recognition.getLeft();
                            } else {
                                stoneTwoX = (int) recognition.getLeft();
                            }
                        }
                        if (skystoneX != -1 && stoneOneX != -1 && stoneTwoX != -1) {
                            if (skystoneX < stoneOneX && skystoneX < stoneTwoX) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                key = 0;
                            } else if (skystoneX > stoneOneX && skystoneX > stoneTwoX) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                key = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                key = 1;
                            }
                        }
                        ssX = skystoneX;
                        sOneX = stoneOneX;
                        sTwoX = stoneTwoX;
                    }

                    //if 2 minerals detected, split into (not) see gold ifs
                    if (updatedRecognitions.size() == 2) {
                        int skystoneX = -1;
                        int stoneOneX = -1;
                        int stoneTwoX = -1;
                        //populates coordinate values
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals("Skystone")) {
                                skystoneX = (int) recognition.getLeft();
                            } else if (stoneOneX == -1) {
                                stoneOneX = (int) recognition.getLeft();
                            } else {
                                stoneTwoX = (int) recognition.getLeft();
                            }
                        }

                        //combination of 1 gold and 1 silver

                        if (skystoneX != -1 && (stoneOneX != -1 || stoneTwoX != -1)) {
                            if (skystoneX < stoneOneX && skystoneX < 700) {
                                telemetry.addData("Gold Mineral Position", "Left (GS)");
                                key = 0;
                            } else if (skystoneX > stoneOneX && skystoneX > 900) {
                                telemetry.addData("Gold Mineral Position", "Right (GS)");
                                key = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center (GS)");
                                key = 1;
                            }
                            // combination of 2 silvers
                        } else if (stoneOneX != -1 && stoneTwoX != -1) {
                            if ((200 < stoneOneX) && (stoneOneX < 300) && (350 < stoneTwoX)) {
                                telemetry.addData("Gold Mineral Position", "Left (SS1)");
                                key = 0;
                            } else if ((200 < stoneTwoX) && (stoneTwoX < 300) && (350 < stoneOneX)) {
                                telemetry.addData("Gold Mineral Position", "Left (SS2)");
                                key = 0;
                            } else if ((150 > stoneOneX) && ((200 < stoneTwoX) && (stoneTwoX < 300))) {
                                telemetry.addData("Gold Mineral Position", "Right (SS3)");
                                key = 2;
                            } else if ((150 > stoneTwoX) && ((200 < stoneOneX) && (stoneOneX < 300))) {
                                telemetry.addData("Gold Mineral Position", "Right (SS4)");
                                key = 2;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center (SS5)");
                                key = 1;
                            }
                        }

                        ssX = skystoneX;
                        sOneX= stoneOneX;
                        sTwoX = stoneTwoX;
                    }
                    telemetry.addData("Key", key);
                    telemetry.addData("skystoneX", ssX);
                    telemetry.addData("stoneOneX", sOneX);
                    telemetry.addData("stoneTwoX", sTwoX);
                    telemetry.update();
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
            tfod.deactivate();

        }
        //L440
        //C660
        //R

        if (key == 0) {
            telemetry.addData("gold mineral", "left");
        } else if (key == 1) {
            telemetry.addData("gold mineral", "center");
        } else if (key == 2) {
            telemetry.addData("gold mineral", "right");
        } else if (key == 4) {
            key = 1;
            telemetry.addData("GOLD DETECTION FAILURE", "OVERRIDE KEY 1");
        }
        telemetry.update();

//        encoderAccessory(1,75);

        if (key == 0) {
//            encoderAccessory(1,-75);
//            robot.pivot.setPower(.2);
            encoderDrive(1,400,300,.5);
            encoderDrive(1,1200,0,1);
            robot.intakeFlipperRight.setPosition(.980);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < .5) {}
            robot.intakeFlipperLeft.setPosition(.65);
            robot.pivot.setPower(0);
            encoderDrive(1,600,600,1);
            robot.intakeFlipperLeft.setPosition(.65);
            robot.intakeFlipperRight.setPosition(.985);
            encoderIntake(.6,-300,600,2);
            robot.intakeFlipperLeft.setPosition(.6);
            robot.intakeFlipperRight.setPosition(.9);
            robot.intakeLeft.setPower(0);
            robot.intakeRight.setPower(0);
            robot.intakeFlipperLeft.setPosition(.65);
            robot.intakeFlipperRight.setPosition(.985);
            encoderAccessory(1,300);
            robot.intakeFlipperLeft.setPosition(.5);
            robot.intakeFlipperRight.setPosition(.8);
            encoderDrive(1,-500,-500,.5);
            encoderDrive(.8,0,-860,1.5);
            robot.intakeFlipperRight.setPosition(.980);
            runtime.reset();
            while (opModeIsActive() && runtime.seconds() < .5) {}
            robot.intakeFlipperLeft.setPosition(.65);
            encoderAccessorySpecial(1,75);
            encoderDrive(1,-2300,-2300,1.5);
            encoderDrive(.8,0,-1100,1);
            runtime.reset();
            robot.foundationMover.setDirection(Servo.Direction.FORWARD);
            robot.foundationMover.setPosition(8);
            encoderDrive(1,-400,-400,1);
            while (opModeIsActive() && runtime.seconds() < 2) {
            }
            encoderDrive(1,300,300,1);
            encoderAccessory(1,500);
            encoderDrive(.8,0,-400,1);
            encoderDrive(1,750,750,1);
            encoderDrive(.8,0,1000,1);
            robot.openerRight.setPosition(0);
            robot.openerLeft.setPosition(0);
            encoderDrive(1,-600,-600,1);
            while (opModeIsActive() && runtime.seconds() < 1) {
                robot.foundationMover.setPosition(1);
            }



        } else if (key == 1) {
            encoderDrive(.5,-50,-50,1);
            encoderDrive(.5,-50,-50,1);
        } else if (key == 2) {
            encoderDrive(.5,-50,-50,1);
            encoderDrive(.5,-50,-50,1);
            encoderDrive(.5,-50,-50,1);
        }


    }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.65;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

//    public void encoderPivot(double speed, double encoder, double timeoutS) {
//        int newTarget;
//        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newTarget = robot.pivot.getCurrentPosition() + (int)(-encoder);// * COUNTS_PER_INCH);
//
//            // Turn On RUN_TO_POSITION
//            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//// reset the timeout time and start motion.
//            runtime.reset();
//            robot.pivot.setPower(Math.abs(speed));
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.frontLeft.isBusy() || robot.backLeft.isBusy() || robot.frontRight.isBusy() || robot.backRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.frontLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//        }
//    }

    public void encoderDrive(double speed, double leftEncoder, double rightEncoder, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(-leftEncoder);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightEncoder);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftEncoder);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.frontRight.getCurrentPosition() + (int)(-rightEncoder);// * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newRearLeftTarget);
            robot.backRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

// reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() || robot.backLeft.isBusy() || robot.frontRight.isBusy() || robot.backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

        }
    }

    public void encoderAccessorySpecial(double speed, double encoderAmount) {
        int newSlideTarget;
        int newPivotTarget;
        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newPivotTarget = robot.pivot.getCurrentPosition() + (int)(-encoderAmount);// * COUNTS_PER_INCH);
            robot.pivot.setTargetPosition(newPivotTarget);

            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.pivot.setPower(Math.abs(Math.abs(speed)));

//            while (opModeIsActive() && (robot.pivot.isBusy())) {}
            robot.pivot.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Stop all motion;
//            robot.pivot.setPower(0);

            // Turn off RUN_TO_POSITION
//            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderAccessory(double speed, double encoderAmount) {
        int newSlideTarget;
        int newPivotTarget;
        robot.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newPivotTarget = robot.pivot.getCurrentPosition() + (int)(-encoderAmount);// * COUNTS_PER_INCH);
            robot.pivot.setTargetPosition(newPivotTarget);

            robot.pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.pivot.setPower(Math.abs(Math.abs(speed)));

            while (opModeIsActive() && (robot.pivot.isBusy())) {}
            robot.pivot.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            // Stop all motion;
//            robot.pivot.setPower(0);

            // Turn off RUN_TO_POSITION
//            robot.pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderIntake(double speed, double encoderValue, double travel, double timeoutS) {
        int newLeftTargetIntake;
        int newRightTargetIntake;
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;

        robot.intakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.intakeRight.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.intakeLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTargetIntake = robot.frontLeft.getCurrentPosition() + (int)(encoderValue);// * COUNTS_PER_INCH);
            newRightTargetIntake = robot.frontRight.getCurrentPosition() + (int)(encoderValue);// * COUNTS_PER_INCH);
            robot.intakeLeft.setTargetPosition(newLeftTargetIntake);
            robot.intakeRight.setTargetPosition(newRightTargetIntake);

            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(-travel);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(travel);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(travel);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.frontRight.getCurrentPosition() + (int)(-travel);// * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newRearLeftTarget);
            robot.backRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.intakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.intakeRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.intakeLeft.setPower(Math.abs(speed));
            robot.intakeRight.setPower(Math.abs(speed));

            robot.frontLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.backRight.setPower(Math.abs(speed));

//            if (timeoutS == runtime.seconds()) {
//                robot.intakeLeft.setPower(0);
//                robot.intakeRight.setPower(0);
//            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && (robot.intakeLeft.isBusy() || robot.intakeRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTargetIntake,  newRightTargetIntake);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

        }
    }
    void composeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        */

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        pitch = formatAngle(angles.angleUnit, angles.thirdAngle);
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        /*telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }h
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

        telemetry.addLine()
                .addData("Debugging: ", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        */
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


}
