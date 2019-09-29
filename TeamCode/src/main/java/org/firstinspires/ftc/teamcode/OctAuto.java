
package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.OctHardware;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import java.util.Locale;

@Autonomous(name="OctAuto", group="Autos")
//@Disabled
public abstract class OctAuto extends LinearOpMode {

    OctHardware robot = new OctHardware();
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    public String pitch;

    private static final String VUFORIA_KEY = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeft.getCurrentPosition(),
                robot.frontRight.getCurrentPosition(), robot.backLeft.getCurrentPosition(), robot.backRight.getCurrentPosition());
        telemetry.update();

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia(0);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking - U R GO, GOOD LUCK!");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }

    public void encoderTurn(double baseSpeed, double leftAmount, double rightAmount) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;
        int greater = 0;
        double speedRatio = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if (rightAmount > leftAmount) {
                greater = 1;
            } else if (rightAmount < leftAmount) {
                greater = -1;
            }

            if (greater == 1) {
                speedRatio = rightAmount/leftAmount;
            } else if (greater == -1) {
                speedRatio = leftAmount/rightAmount;
            }

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftAmount);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightAmount);// * COUNTS_PER_INCH);
            newRearLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftAmount);// * COUNTS_PER_INCH);
            newRearRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightAmount);// * COUNTS_PER_INCH);
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
            if (greater == 1) {
                robot.frontLeft.setPower(Math.abs(baseSpeed));
                robot.frontRight.setPower(Math.abs(baseSpeed) * speedRatio);
                robot.backLeft.setPower(Math.abs(baseSpeed));
                robot.backRight.setPower(Math.abs(baseSpeed) * speedRatio);
            } else if (greater == -1) {
                robot.frontLeft.setPower(Math.abs(baseSpeed) * speedRatio);
                robot.frontRight.setPower(Math.abs(baseSpeed));
                robot.backLeft.setPower(Math.abs(baseSpeed) * speedRatio);
                robot.backRight.setPower(Math.abs(baseSpeed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.frontLeft.isBusy() || robot.frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("yeet", robot.frontLeft.getCurrentPosition());
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }


    public void encoderDrive(double speed, double leftEncoder, double rightEncoder, double timeoutS) {
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftEncoder);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightEncoder);// * COUNTS_PER_INCH);
            newBackLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftEncoder);// * COUNTS_PER_INCH);
            newBackRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightEncoder);// * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);

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

        }
    }

    public void proportionalDrive(double speed, double leftEncoder, double rightEncoder, double timeoutS) {
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftEncoder);// * COUNTS_PER_INCH);
            newFrontRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightEncoder);// * COUNTS_PER_INCH);
            newBackLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftEncoder);// * COUNTS_PER_INCH);
            newBackRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightEncoder);// * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newFrontLeftTarget);
            robot.frontRight.setTargetPosition(newFrontRightTarget);
            robot.backLeft.setTargetPosition(newBackLeftTarget);
            robot.backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double progress = robot.frontLeft.getCurrentPosition()/newFrontLeftTarget;
            double pSpeed = speed * progress;

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeft.setPower(Math.abs(pSpeed));
            robot.frontRight.setPower(Math.abs(pSpeed));
            robot.backLeft.setPower(Math.abs(pSpeed));
            robot.backRight.setPower(Math.abs(pSpeed));

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

        }
    }

//    public void encoderAccessory(double speed, double encoderAmount, int port) {
//        int newSlideTarget;
//        int newPivotTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            if (port == 0) {
//                newSlideTarget = robot.hexSlide.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
//                robot.hexSlide.setTargetPosition(newSlideTarget);
//            } else if (port == 1) {
//                newPivotTarget = robot.pivotMotor.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
//                robot.pivotMotor.setTargetPosition(newPivotTarget);
//            }
//
//            // Turn On RUN_TO_POSITION
//            if (port == 0) {
//                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if (port == 1) {
//                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            if (port == 0) {
//                robot.hexSlide.setPower(Math.abs(speed));
//            } else if (port == 1) {
//                robot.pivotMotor.setPower(Math.abs(speed));
//            }
//
//            while (opModeIsActive() && (robot.hexSlide.isBusy() || robot.pivotMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running at %7d :%7d", robot.hexSlide.getCurrentPosition(), robot.pivotMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.hexSlide.setPower(0);
//            robot.pivotMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }

//    public void encoderAccessoryTimeout(double speed, double encoderAmount, int port, double timeoutS) {
//        int newSlideTarget;
//        int newPivotTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            if (port == 0) {
//                newSlideTarget = robot.hexSlide.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
//                robot.hexSlide.setTargetPosition(newSlideTarget);
//            } else if (port == 1) {
//                newPivotTarget = robot.pivotMotor.getCurrentPosition() + (int)(encoderAmount);// * COUNTS_PER_INCH);
//                robot.pivotMotor.setTargetPosition(newPivotTarget);
//            }
//
//            // Turn On RUN_TO_POSITION
//            if (port == 0) {
//                robot.hexSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            } else if (port == 1) {
//                robot.pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            }
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            if (port == 0) {
//                robot.hexSlide.setPower(Math.abs(speed));
//            } else if (port == 1) {
//                robot.pivotMotor.setPower(Math.abs(speed));
//            }
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.hexSlide.isBusy() || robot.pivotMotor.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running at %7d :%7d", robot.hexSlide.getCurrentPosition(), robot.pivotMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.hexSlide.setPower(0);
//            robot.pivotMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.hexSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }

    private void initVuforia(double io) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        if (io == 0) {

            parameters.vuforiaLicenseKey = "AVF7OF7/////AAABmaKBSYRMHkclubr6nFb2TLcr3QzadzX163OzDe2NS0p2hQlEvibYh8W2xO78LrAUPInfApVZ1qzOxq7fnHZ9KQ0QiJM0E5WbwxdY7U+Gbrk8NuDgceoPw7eD8j2Sk7NuvuTcXYAAoA4wKwgDlw+iA19frB/9/WuUonCWiMAi+sxSoAGkudWAx8f1AO0AXBNyf6d0QHRGVeGRyMYtvkvsez3kU6U7LnMUwpDkX5RfQi+AMKq+BTLYtOo90waG5G84TV9LU1OSlDHtPh7sSG6YuVdn0Pmm/+k9nEtedozzDeDmwKfT1A5uL1m+RGmgCe4gA45H7qH6p9ymyKDbhvfDbTo/fVI0Y9g+Z8FEMByMnI6X";
            parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

        } else if (io == 1) {
            parameters.camera.close();
        }
        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

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

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
