package org.firstinspires.ftc.teamcode;

import android.content.res.AssetFileDescriptor;
import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AnimatornicsRobot {

    private DcMotor lfMotor;
    private DcMotor lbMotor;
    private DcMotor rfMotor;
    private DcMotor rbMotor;

    private DcMotor liftMotor_1;
    private DcMotor liftMotor_2;
    private DcMotor slideMotor;

    //private Servo leftServo;
    //private Servo rightServo;
    private CRServo leftServo;
    private CRServo rightServo;
    private CRServo teamMarkerServo;

    private Telemetry telemetry;

    private boolean holdLift = false;
    private boolean turnOnlyOneServo = false;
    private ElapsedTime runtime = new ElapsedTime();

    private MediaPlayer player = null;
    private AssetFileDescriptor helloDescriptor = null;
    private AssetFileDescriptor byeDescriptor = null;

    /*
     * Going forward: lb:-1, lf:-1, rf:1, rb:1
     * Going reverse: lb:1, lf:1, rf:-1, rb:-1
     * Going turn right: lb:-1, lf:-1, rf:-1, rb:-1
     * Going turn left: lb:1, lf:1, rf:1, rb:1
     * Going lateral right: lb:-1, lf:1, rf:-1, rb:1
     * Going lateral left: lb:1, lf:-1, rf:1, rb:-1
     */

    public AnimatornicsRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;
        lfMotor = hardwareMap.get(DcMotor.class, "lfmotor");
        lbMotor = hardwareMap.get(DcMotor.class, "lbmotor");
        rfMotor = hardwareMap.get(DcMotor.class, "rfmotor");
        rbMotor = hardwareMap.get(DcMotor.class, "rbmotor");

        slideMotor = hardwareMap.get(DcMotor.class, "slide_motor");
        liftMotor_1 = hardwareMap.get(DcMotor.class, "lift_motor_1");
        liftMotor_2 = hardwareMap.get(DcMotor.class, "lift_motor_2");

        //leftServo = hardwareMap.get(Servo.class, "leftServo");
        //rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        teamMarkerServo = hardwareMap.get(CRServo.class, "teamMarkerServo");

        liftMotor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        telemetry.addData("Status", "DC motor variables initialized");
        System.out.println("slideMotor controller = " + slideMotor.getController());
        System.out.println("liftMotor_1 controller = " + liftMotor_1.getController());
        System.out.println("liftMotor_2 controller = " + liftMotor_2.getController());

        try {
            helloDescriptor = hardwareMap.appContext.getAssets().openFd("Hello.mp3");
            byeDescriptor = hardwareMap.appContext.getAssets().openFd("ByeBye.mp3");
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }

    public void manualDrive(LinearOpMode op) {

        double lfPower = op.gamepad1.left_stick_y - op.gamepad1.left_stick_x;
        double lbPower = op.gamepad1.left_stick_y + op.gamepad1.left_stick_x;
        double rfPower = -(op.gamepad1.right_stick_y + op.gamepad1.left_stick_x);
        double rbPower = -(op.gamepad1.right_stick_y - op.gamepad1.left_stick_x);

        lfPower = Range.clip(lfPower, -1, 1);
        lbPower = Range.clip(lbPower, -1, 1);
        rfPower = Range.clip(rfPower, -1, 1);
        rbPower = Range.clip(rbPower, -1, 1);

        lfMotor.setPower(lfPower);
        lbMotor.setPower(lbPower);
        rfMotor.setPower(rfPower);
        rbMotor.setPower(rbPower);

        telemetry.addData("Status", "lfPower:"+lfPower+", lbPower:"+lbPower+", rfPower:"+rfPower+", rbPower:"+rbPower);

        if(op.gamepad1.y) {
            holdLift = true;
        }
        if(op.gamepad1.x) {
            holdLift = false;
        }

        if(!holdLift) {
            double liftPower =  op.gamepad1.left_trigger - op.gamepad1.right_trigger;
            liftPower = liftPower; // full power is too fast.
            liftMotor_1.setPower(liftPower);
            liftMotor_2.setPower(liftPower);
            telemetry.addData("Status", "liftPower: " + liftPower);
            //System.out.println("SaiC: liftPower="+liftPower+", position_1="+liftMotor_1.getCurrentPosition()+", position_2="+liftMotor_2.getCurrentPosition());
        } else {
            double liftPower = 0.5;
            liftMotor_1.setPower(liftPower);
            liftMotor_2.setPower(liftPower);
            telemetry.addData("Status", "liftPower: " + liftPower);
        }

        double slidePower = op.gamepad2.left_stick_y+op.gamepad2.right_stick_y;
        slidePower = Range.clip(slidePower, -1, 1);
        slidePower = slidePower; // full power is too fast.
        slideMotor.setPower(slidePower);
        telemetry.addData("Status", "slidePower: " + slidePower);
        //System.out.println("SaiC: slidePower="+slidePower+", position="+slideMotor.getCurrentPosition());


        /*double leftServoPower = op.gamepad2.left_trigger;
        double rightServoPower = op.gamepad2.right_trigger;
        if(leftServoPower < 0.75) {
            leftServo.setPosition(1.0);
        } else {
            leftServo.setPosition(0.0);
        }

        if(rightServoPower > 0.75) {
            rightServo.setPosition(1.0);
        } else {
            rightServo.setPosition(0.0);
        }*/

        if(op.gamepad2.y) {
            turnOnlyOneServo = true;
        }
        if(op.gamepad2.x) {
            turnOnlyOneServo = false;
        }

        if(!turnOnlyOneServo) {
            double crServoPower = op.gamepad2.left_trigger - op.gamepad2.right_trigger;
            leftServo.setPower(crServoPower);
            rightServo.setPower(-crServoPower);
        } else {
            double crServoPower = op.gamepad2.left_trigger - op.gamepad2.right_trigger;
            leftServo.setPower(crServoPower);
        }

        if(op.gamepad1.a || op.gamepad1.b) {
            try {
                if(player != null) {
                    if(player.isPlaying()) player.stop();
                    player.release();
                }
                player = new MediaPlayer();
                if(op.gamepad1.a)
                    player.setDataSource(helloDescriptor.getFileDescriptor(), helloDescriptor.getStartOffset(), helloDescriptor.getLength());
                else if(op.gamepad1.b)
                    player.setDataSource(byeDescriptor.getFileDescriptor(), byeDescriptor.getStartOffset(), byeDescriptor.getLength());
                player.prepare();
                player.setVolume(1f, 1f);
                player.setLooping(false);
                player.start();
            } catch (Exception ex) {
                ex.printStackTrace();
            }
        }

        if(op.gamepad2.a && op.gamepad2.b) {
            teamMarkerServo.setPower(0.0);
        } else if (op.gamepad2.a){
            teamMarkerServo.setPower(1.0);
        } else if(op.gamepad2.b) {
            teamMarkerServo.setPower(-1.0);
        }

    }

    public void moveLift(LinearOpMode op, double time, String direction, double liftPower) {

        liftMotor_1.setPower(liftPower);
        liftMotor_2.setPower(liftPower);
        runtime.reset();
        while (op.opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Lift:"+direction+": %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        liftMotor_1.setPower(0.0);
        liftMotor_2.setPower(0.0);
    }

    public void setLiftPower(LinearOpMode op, double liftPower) {
        liftMotor_1.setPower(liftPower);
        liftMotor_2.setPower(liftPower);
    }

    public void moveRobot(LinearOpMode op, double time, String direction, double lfPower, double lbPower, double rfPower, double rbPower) {

        lfMotor.setPower(lfPower);
        lbMotor.setPower(lbPower);
        rfMotor.setPower(rfPower);
        rbMotor.setPower(rbPower);
        runtime.reset();
        while (op.opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Robot:"+direction+": %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        lfMotor.setPower(0.0);
        lbMotor.setPower(0.0);
        rfMotor.setPower(0.0);
        rbMotor.setPower(0.0);
    }

    public void setWheelPower(double lfPower, double lbPower, double rfPower, double rbPower) {

        lfMotor.setPower(lfPower);
        lbMotor.setPower(lbPower);
        rfMotor.setPower(rfPower);
        rbMotor.setPower(rbPower);
    }

    public void moveSlide(LinearOpMode op, double time, String direction, double slidePower) {

        slideMotor.setPower(slidePower);
        runtime.reset();
        while (op.opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Slide:"+direction+": %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        slideMotor.setPower(0.0);
    }

    public void setCollectorPower(LinearOpMode op, double time, String direction, double crServoPower) {

        leftServo.setPower(crServoPower);
        rightServo.setPower(-crServoPower);
        runtime.reset();
        while (op.opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Collector:"+direction+": %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
    }

    public void turnTeamMarkerServo(LinearOpMode op, double time, String direction, double tmPower) {

        teamMarkerServo.setPower(tmPower);
        runtime.reset();
        while (op.opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "TM:"+direction+": %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        teamMarkerServo.setPower(0.0);
    }
}
