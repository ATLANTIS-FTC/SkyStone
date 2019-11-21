package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

@TeleOp
//@Disabled
public class Intake extends LinearOpMode {

    OctHardware robot = new OctHardware();
    private ElapsedTime runtime = new ElapsedTime();

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    public String pitch;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        boolean slowMode = false;
        boolean reverse = false;
        boolean open = false;
        int pivotPos = 0;

        while (opModeIsActive()) {

            //gamepad 1
            double throttle = ((gamepad1.right_trigger) - (gamepad1.left_trigger));
            double steering = gamepad1.left_stick_x;

            if (gamepad1.a) {
                slowMode = !slowMode;
                while (gamepad1.a);
            }

            double rPower = throttle - steering;
            double lPower = throttle + steering;

            if (slowMode == true) {
                if (steering != 0) {
                    lPower /= 2.5;
                    rPower /= 2.5;
                } else {
                    lPower /= 2;
                    rPower /= 2;
                }
            }

            if (gamepad1.right_bumper) {
                lPower *= 2;
                rPower *= 2;
            }

            //gamepad 2
            double pivot = -gamepad2.left_stick_y;
            double intake = ((-gamepad2.right_trigger) + (gamepad2.left_trigger));

            if (gamepad2.right_bumper) {
                pivot *= 3;
            }

            if (gamepad2.b) {
                robot.intakeFlipperLeft.setPosition(.5);
                robot.intakeFlipperRight.setPosition(.8);
            }

            if (gamepad2.y) {
                robot.intakeFlipperLeft.setPosition(.6);
                robot.intakeFlipperRight.setPosition(.9);
            }

            if (gamepad2.x) {
                robot.intakeFlipperLeft.setPosition(.2);
                robot.intakeFlipperRight.setPosition(.5);
            }

            if (gamepad2.dpad_up) {
                robot.openerRight.setPosition(.2);
                robot.openerLeft.setPosition(.15);
            }

            if (gamepad2.dpad_down) {
                robot.openerRight.setPosition(0);
                robot.openerLeft.setPosition(0);
            }

            if (gamepad2.dpad_left) {
                robot.foundationMover.setPosition(1);
            }

            if (gamepad2.dpad_right) {
                robot.foundationMover.setPosition(.5);
            }


            //gamepad 1 setPower
            robot.frontLeft.setPower(-lPower/3);
            robot.frontRight.setPower(rPower/3);
            robot.backLeft.setPower(lPower/3);
            robot.backRight.setPower(-rPower/3);

            telemetry.addData("lpower", lPower);
            telemetry.addData("rpower", rPower);
            telemetry.addData("Status", "Running");
            telemetry.addData("slowmode", slowMode);
            telemetry.update();

            //gamepad 2 setPower
            robot.intakeRight.setPower(intake);
            robot.intakeLeft.setPower(-intake);
            robot.pivot.setPower(pivot/4);

        }
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

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
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
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
