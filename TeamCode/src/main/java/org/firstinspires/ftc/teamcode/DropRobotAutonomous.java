package org.firstinspires.ftc.teamcode;


import android.widget.Space;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.ObjectTracker;
import com.vuforia.TrackerManager;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

public abstract class DropRobotAutonomous extends LinearOpMode {


    private static final String VUFORIA_KEY = "AXx8B3P/////AAABmUvCh01qr0NYuiZsl4XR5wxHpJiI+AQBbtiTScffb3UpHwbjqT0gTnTtblgQyH6abrP5hIOA/Y4wgs8bU+1LwD/bor01NOM30m6KKqBS0hrGh0Z8IZu+1sNQyNzgm5dZNUKFI7UzEGUTlEL0L8r1v2++74NQkE8ZZFs6WyUjEowkDBpYQQE0ANXA5qDl0g2Rd7S3Y4rk9HgRJrJaZ0ojGT0uNzHdjkO7gpPYFsEDfAPVz7Pguzw7psyDlPvRmnKajnomWiCVEortJir77e1fgPSCnLobhrXL8b9PN3vaLu8ow0GxMbmJJN1ni0m+vzguiRaNy4JhDDgevKJuN4bv5CLqIt1EDMDG9ROrxcq3OJMR";
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static String Blue_Rover = "Blue-Rover";
    private static String Red_Footprint = "Red-Footprint";
    private static String Front_Craters = "Front-Craters";
    private static String Back_Space = "Back-Space";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
    private final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
    private final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

    private final boolean FIND_NAV_TARGETS = false;

    private AnimatornicsRobot robot = null;
    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod = null;
    private boolean targetVisible = false;
    private String targetName = null;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean isGoldDetected = false;
    private Recognition goldRecognition = null;
    private int goldPosition = Position.NONE;
    private VuforiaTrackables targetsRoverRuckus = null;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    interface Position {
        public static int NONE = 0, RIGHT = 1, CENTER = 2, LEFT = 3;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new AnimatornicsRobot(hardwareMap, telemetry);

        if(FIND_NAV_TARGETS) {
            initVuforia();
            loadNavigationTragets();

            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();
        } else {
            initVuforiaTF();
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                telemetry.update();
            }
            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();
        }
        // start lateral left
        robot.setWheelPower(0.5, -0.5, 0.5, -0.5);
        // Drop the robot to ground
        robot.moveLift(this, 1.6, "Drop", -1.0);
        // stop lateral left
        robot.setWheelPower(0.0, 0.0, 0.0, 0.0);

        if(FIND_NAV_TARGETS) {
            targetsRoverRuckus.activate();
            targetVisible = false;
        }
        robot.moveRobot(this, 1.0, "Turn-Right",-0.2, -0.2, -0.2, -0.2);
        robot.setWheelPower(0.0, 0.0, 0.0, 0.0);
        telemetry.addData(">", "Done Turning......");
        telemetry.update();
        if(FIND_NAV_TARGETS) {
            Thread.sleep(1000);
            lookForNavigationTarget(allTrackables, 1.0);
            targetsRoverRuckus.deactivate();
        }

        if(FIND_NAV_TARGETS) {
            if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
                initTfod();
            } else {
                telemetry.addData("Sorry!", "This device is not compatible with TFOD");
                telemetry.update();
            }

            telemetry.addData(">", "Press Play to start tracking");
            telemetry.update();
            waitForStart();
        }

        if (tfod != null) {
            tfod.activate();
            checkForGoldAndSilverMinerals(10.0, true);
            if (!isGoldDetected) {
                checkForGoldAndSilverMinerals(6.0, false);
            }

            if (isGoldDetected) {
                trackGoldMineral(10.0);
            } else {
                telemetry.addData("Sorry!!!", "No GOLD mineral");
            }
        } else {
            telemetry.addData("Sorry!!!", "This device is not compatible with TFOD");
            telemetry.update();
        }
        //Thread.sleep(30000);
        if (tfod != null) {
            tfod.shutdown();
        }


        System.out.println("SaiC: goldPosition="+goldPosition);
        if(this instanceof DepoDropRobot || (targetName != null && (targetName.equals(Blue_Rover) || targetName.equals(Red_Footprint)))) {

            if(goldPosition == Position.RIGHT) {
                // Go little forward
                //robot.moveRobot(this, 1.0, "forward",-0.3, -0.3, 0.3, 0.3);
                // Turn left
                robot.moveRobot(this, 1.5, "left",0.3, 0.3, 0.3, 0.3);
            }
            if(goldPosition == Position.LEFT) {
                // Go little forward
                //robot.moveRobot(this, 0.5, "forward",-0.3, -0.3, 0.3, 0.3);
                // Turn right
                robot.moveRobot(this, 1.5, "right",-0.3, -0.3, -0.3, -0.3);
            }

            // Go close to the depo
            robot.moveRobot(this, 3.0, "forward",-0.3, -0.3, 0.3, 0.3);
            // extend Slide up
            robot.moveSlide(this, 5.0, "up", -1.0);
            // lift up
            robot.moveLift(this, 4.0, "Close", 1.0);
            // drop team marker
            robot.setCollectorPower(this, 4.0, "Out", -1.0);
        } else {
            if(goldPosition == Position.RIGHT) {
                // Go little forward
                //robot.moveRobot(this, 1.0, "forward",-0.3, -0.3, 0.3, 0.3);
                // Turn left
                robot.moveRobot(this, 0.5, "left",0.3, 0.3, 0.3, 0.3);
            }
            if(goldPosition == Position.LEFT) {
                // Go little forward
                //robot.moveRobot(this, 0.5, "forward",-0.3, -0.3, 0.3, 0.3);
                // Turn right
                robot.moveRobot(this, 0.5, "right",-0.3, -0.3, -0.3, -0.3);
            }
            // Go close to the crater
            //robot.moveRobot(this, 1.0, "forward",-0.3, -0.3, 0.3, 0.3);
            // extend Slide
            robot.moveSlide(this, 5.0, "up", -1.0);
            // Drop slide in to the crater
            robot.moveLift(this, 3.0, "Close", 1.0);
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    private void initVuforiaTF() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void loadNavigationTragets() {
        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName(Blue_Rover);
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName(Red_Footprint);
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName(Front_Craters);
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName(Back_Space);

        allTrackables.addAll(targetsRoverRuckus);
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, CAMERA_CHOICE);
        }
    }

    private void lookForNavigationTarget(List<VuforiaTrackable> allTrackables, double time) {
        runtime.reset();
        while (opModeIsActive() && !targetVisible && runtime.seconds() < time) {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    targetVisible = true;
                    targetName = trackable.getName();
                }
            }

            if(targetVisible) {
                telemetry.addData("Visible Target", targetName);
                // TODO: Stop robot
                robot.setWheelPower(0.0, 0.0, 0.0, 0.0);
            } else {
                telemetry.addData("Visible Target", "none");
                // TODO: turn robot to right until you see any target
                robot.setWheelPower(-0.1, -0.1, -0.1, -0.1);
            }
            telemetry.update();
        }
    }

    private void checkForGoldAndSilverMinerals(double time, boolean leftSide) {
        runtime.reset();
        int prevNumberOfObjectDetected = 0;
        boolean firstTwoAreSilver = false;
        while(opModeIsActive() && !isGoldDetected && runtime.seconds() < time) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                int numberOfObjectDetected = updatedRecognitions.size();
                telemetry.addData("# Object Detected", numberOfObjectDetected);
                telemetry.addData("Visible Target", targetName);

                if (numberOfObjectDetected == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            isGoldDetected = true;
                            goldRecognition = recognition;
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            goldPosition = Position.LEFT;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            goldPosition = Position.RIGHT;
                        } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                            goldPosition = Position.CENTER;
                        }
                    }
                } else if (numberOfObjectDetected == 2) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            isGoldDetected = true;
                            goldRecognition = recognition;
                            goldMineralX = (int) recognition.getLeft();
                        } else {
                            silverMineral1X = (int) recognition.getLeft();
                        }
                    }
                    if(!isGoldDetected) {
                        firstTwoAreSilver = true;
                        telemetry.addData("Gold Mineral Position", "Center");
                        goldPosition = Position.LEFT;
                    } else {
                        if(!firstTwoAreSilver) {
                            if (goldMineralX != -1 && silverMineral1X != -1) {
                                if (goldMineralX < silverMineral1X) {
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    goldPosition = Position.CENTER;
                                } else {
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    goldPosition = Position.RIGHT;
                                }
                            }
                            if(runtime.seconds() > 5) { // may be first it detected single silver
                                goldPosition++;
                            }
                        }
                    }
                }
                else if (numberOfObjectDetected == 1) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldPosition++;
                            isGoldDetected = true;
                            goldRecognition = recognition;
                            telemetry.addData("# Object Detected", "GOLD");
                            break;
                        }
                    }
                    if(!isGoldDetected && prevNumberOfObjectDetected == 0) {
                        goldPosition++;
                    }
                }
                System.out.println("SaiC: prevNumberOfObjectDetected="+prevNumberOfObjectDetected+", numberOfObjectDetected="+numberOfObjectDetected+", goldPosition="+goldPosition);
                prevNumberOfObjectDetected = numberOfObjectDetected;
            }

            if(isGoldDetected) {
                // TODO: Stop Robot and try moving forward to hit Gold minaral
                robot.setWheelPower(0.0, 0.0, 0.0, 0.0);
            } else {
                if(leftSide) {
                    robot.setWheelPower(0.15, 0.15, 0.15, 0.15);
                } else {
                    robot.setWheelPower(-0.15, -0.15, -0.15, -0.15);
                }
            }
            telemetry.update();
        }
    }

    private void trackGoldMineral(double time) {
        runtime.reset();
        boolean isDone = false;
        while(opModeIsActive() && !isDone && runtime.seconds() < time) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                telemetry.addData("Visible Target", targetName);
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        isGoldDetected = true;
                        goldRecognition = recognition;
                        telemetry.addData("# Object Detected", "GOLD");
                        break;
                    }
                }
            }

            if (goldRecognition != null) {
                float goldMineralSizeInCamera = goldRecognition.getWidth();
                double angle =  goldRecognition.estimateAngleToObject(AngleUnit.DEGREES);
                telemetry.addData("Gold Mineral Position", "width:" + goldMineralSizeInCamera +
                        ", angle:" + angle + ", IWidth:" + goldRecognition.getImageWidth());

                if(angle > 4) {
                    //TODO: trun right slowly
                    robot.setWheelPower(-0.15, -0.15, -0.15, -0.15);
                } else if (angle < -4) {
                    // TODO: turn left slowly
                    robot.setWheelPower(0.15, 0.15, 0.15, 0.15);
                } else {
                    if(goldMineralSizeInCamera < goldRecognition.getImageWidth()) {
                        // TODO: Move straight
                        robot.setWheelPower(-0.15, -0.15, 0.15, 0.15);
                    } else {
                        // TODO: Stop?
                        isDone = true;
                        telemetry.addData("Gold Mineral Position", "HIT.....");
                    }
                }
            } else {
                telemetry.addData("Gold Mineral Position", "Lost it what to do now?");
                //TODO: Stop?
                isDone = true;
            }
            telemetry.update();
        }
    }
}
