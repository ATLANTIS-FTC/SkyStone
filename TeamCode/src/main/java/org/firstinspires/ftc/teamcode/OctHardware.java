package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class OctHardware {
    /* Public OpMode members. */
    public DcMotor  backLeft  = null;
    public DcMotor  backRight  = null;
    public DcMotor  frontLeft  = null;
    public DcMotor  frontRight  = null;
    public DcMotor  intakeLeft  = null;
    public DcMotor  intakeRight  = null;
    public DcMotor pivot = null;

    public Servo intakeFlipperLeft = null;
    public Servo intakeFlipperRight = null;
    public Servo foundationMover = null;
    public Servo openerRight = null;
    public Servo openerLeft = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public OctHardware (){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        backLeft  = hwMap.get(DcMotor.class, "back_left");
        frontLeft  = hwMap.get(DcMotor.class, "front_left");
        backRight = hwMap.get(DcMotor.class, "back_right");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        intakeLeft = hwMap.get(DcMotor.class, "intake_left");
        intakeRight = hwMap.get(DcMotor.class, "intake_right");
        pivot = hwMap.get(DcMotor.class, "pivot");

        //Define and Initialize Servos
        intakeFlipperLeft  = hwMap.get(Servo.class, "intake_flipper_left");
        intakeFlipperRight  = hwMap.get(Servo.class, "intake_flipper_right");
        foundationMover = hwMap.get(Servo.class, "foundation_mover");
        openerRight = hwMap.get(Servo.class,"opener_right");
        openerLeft = hwMap.get(Servo.class,"opener_left");
        intakeFlipperRight.setDirection(Servo.Direction.REVERSE);
        foundationMover.setDirection(Servo.Direction.REVERSE);
        openerRight.setDirection(Servo.Direction.REVERSE);
        openerLeft.setDirection(Servo.Direction.FORWARD);

        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        intakeRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark
        pivot.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        backLeft.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        pivot.setPower(0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
}

