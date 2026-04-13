package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Configurable
@Autonomous(name = "ppBlueGATENear4Cycle", group = "PP")
// @Autonomous(...) is the other common choice

public class ppBlueGATENear4Cycle extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();
    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
    // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
    // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";
// Set up stuff for pedro path

    private String thisUpdate = "20";
    private TelemetryManager telemetryMU;
    //Private Follower follower;
    public static Follower follower;
    private Timer  actionTimer, opmodeTimer;
    private ElapsedTime pTimer, pathTimer;// this is for pausing at the end of a path
    //configurables for pedro
    public static double powerCreeper = 0.15;
    public  static  double powerSlow = 0.36;
    public static double powerStart = 0.5;
    public static double powerNormal = 0.68;
    public static double powerFast = 0.8;
    // poses for pedropath
    // poses for pedropath
    public static Pose startPose = new Pose(33.5, 134, Math.toRadians(180)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(57, 105, Math.toRadians(143)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP =new Pose(57,105,Math.toRadians(142));
    public static Pose pickup1aPose = new Pose(50, 84.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    //public static Pose gatePose = new Pose(13 , 70,Math.toRadians(-90)) ;   //This pos will have robot hit gate After First Spike
    public static Pose pickup1bPose = new Pose(14, 72, Math.toRadians(180)); // was 84y (First Set) of Artifacts picked up.
    public static Pose pickup2aPose = new Pose(49, 55, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup2bPose = new Pose(6, 51.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public static Pose pickReturn2 =new Pose(20,75,180);
    public static Pose pickup3aPose = new Pose(49, 34, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup3bPose = new Pose(6, 27, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public static Pose endPose = new Pose(45,58,Math.toRadians(180));

    private Pose currentTargetPose = startPose;
    private Pose lastPose = startPose;
    private PathChain scorePreload;
    private PathChain grabPickup1a,gogatePose, grabPickup1b, scorePickup1, grabPickup2a,grabPickup2b, scorePickup2, grabPickup3a,grabPickup3b, scorePickup3, endPath;

    // private Path grabPickup1a;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
     /*   scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setHeadingConstraint(0.1);
        scorePreload.setVelocityConstraint(2.0);*/

        scorePreload = follower.pathBuilder()
                .addPath (new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1a = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())
                .build();
           /*
        gogatePose = follower.pathBuilder()
                .addPath(new BezierLine(pickup1bPose, gatePose))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), gatePose.getHeading())
                .build();
            */

        grabPickup1b = follower.pathBuilder()
                .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1bPose , scorePoseAP))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePose.getHeading()).setHeadingConstraint(0.1)
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2a = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAP, pickup2aPose))
                .setLinearHeadingInterpolation(scorePoseAP.getHeading(), pickup2aPose.getHeading())
                .build();
        grabPickup2b = follower.pathBuilder()
                .addPath(new BezierLine(pickup2aPose, pickup2bPose))
                .setLinearHeadingInterpolation(pickup2aPose.getHeading(), pickup2bPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
      /*  scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2bPose, pickup2aPose))
                .setLinearHeadingInterpolation(pickup2bPose.getHeading(), pickup1aPose.getHeading())
                .addPath(new BezierLine(pickup2bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePoseAP.getHeading())
                .build();*/
        //tring a curve
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2bPose, pickup2aPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup2bPose.getHeading(), scorePoseAP.getHeading())
                // .addPath(new BezierLine(pickup2bPose, scorePoseAP))
                //.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePoseAP.getHeading())
                .build();
        /*
         *///This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. *//*
        grabPickup3a = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAP, pickup3aPose))
                .setLinearHeadingInterpolation(scorePoseAP.getHeading(), pickup3bPose.getHeading())
                .build();
        grabPickup3b = follower.pathBuilder()
                .addPath(new BezierLine(pickup3aPose,pickup3bPose))
                .setLinearHeadingInterpolation(pickup3bPose.getHeading(), pickup3bPose.getHeading())
                .build();

        //This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. *//*
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup3bPose.getHeading(), scorePose.getHeading())
                .build();
        endPath = follower.pathBuilder()
                .addPath(new BezierCurve(scorePoseAP, pickup2aPose, endPose))
                //.setLinearHeadingInterpolation(scorePoseAP.getHeading(), pickup2aPose.getHeading())
                //.addPath(new BezierLine(pickup2aPose, pickup2bPose))
                .setLinearHeadingInterpolation(scorePoseAP.getHeading(),endPose.getHeading())
                .build();
    }




    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
        pathTimer = new ElapsedTime();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pTimer = new ElapsedTime();

        follower =  CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(startPose);
        follower.update();
        //  pedroPanelsTelemetry.init();
        Drawing.init();
        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

        // disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
        // Feedback to Driver Hub for debugging
        updateTelemetry();

    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }


    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
        opmodeTimer.resetTimer();


    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _unknown:
                currentStage = stage._00_preStart;
                break;

            case _00_preStart:
                currentStage = stage._20_DriveToScore;
                break;

            case _20_DriveToScore:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload, powerStart, true);
                    lastPose = startPose;
                    currentTargetPose = scorePose;

                    // follower.update();
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._25_checkDrivetoscore;
                }
            case _25_checkDrivetoscore:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    currentStage = stage._30_Shoot1; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;

            case _30_Shoot1:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("waiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        runtime.reset();
                        currentStage = stage._40_LauncherStop;
                    }}
                break;

            case _40_LauncherStop:
                if (runtime.milliseconds() >= 1350) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._50_Pickup1;
                }
                break;

            case _50_Pickup1:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1a, powerNormal, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup1aPose;
                    currentStage = stage._55_Pickup1_Startintake;
                }
                break;

            case _55_Pickup1_Startintake:
                if (!follower.isBusy()) {
                    // follower.followPath(grabPickup1a, true);
                    currentTargetPose = pickup1aPose;
                    robot.intake.cmdFoward();
                    currentStage = stage._60_Pickup1a;
                }
                break;

            case _60_Pickup1a:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1b,powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup1aPose;
                    pathTimer.reset();
                    currentStage = stage._70_ToScorePoseAP;
                }
                break;
            case _70_ToScorePoseAP:
                if(!follower.isBusy() || pathTimer.milliseconds() >= 2300){
                    follower.followPath(scorePickup1,powerNormal,true);
                    lastPose = currentTargetPose;
                    currentTargetPose = scorePose;
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._75_chkDrive_to_score_P1;
                }
                break;
            case _75_chkDrive_to_score_P1:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    runtime.reset();
                    currentStage = stage._80_ScorePickup1; // we don't need to do the turn since heading is adjusted in path
                }
                break;

            case _80_ScorePickup1:
                if (!follower.isBusy()) {
                    //                   if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                    //                           CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("waiting to shoot 2");
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        runtime.reset();
                        currentStage = stage._90_LauncherStop;
                    }}

            case _90_LauncherStop:
                if (runtime.milliseconds() >= 1350) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._100_Pickup2;
                }
                break;

            case _100_Pickup2:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2a, powerNormal, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup2aPose;
                    currentStage = stage._110_Pickup2_Startintake;
                }
                break;

            case _110_Pickup2_Startintake:
                if (!follower.isBusy()) {
                    // follower.followPath(grabPickup1a, true);
                    currentTargetPose = pickup2aPose;
                    robot.intake.cmdFoward();
                    currentStage = stage._120_Pickupa2;
                }
                break;

            case _120_Pickupa2:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2b ,powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose= pickup2bPose;
                    pathTimer.reset();
                    currentStage = stage._130_ToScorePoseAP;
                }
                break;
            case _130_ToScorePoseAP:
                if(!follower.isBusy() || pathTimer.milliseconds() >= 2450){
                    follower.followPath(scorePickup2,powerFast,true);
                    currentTargetPose = scorePoseAP;
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._140_chkDrive_to_scorePoseAP;
                }
                break;
            case _140_chkDrive_to_scorePoseAP:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    runtime.reset();
                    currentStage = stage._150_ScorePickup2; // we don't need to do the turn since heading is adjusted in path

                }
                break;

            case _150_ScorePickup2:
                if (!follower.isBusy()) {
                    //                   if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                    //                           CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("waiting to shoot 3");
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        runtime.reset();
                        currentStage = stage._155_LauncherStop;
                    }
                }
                break;

            case _155_LauncherStop:
                if (runtime.milliseconds() >= 1400) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    runtime.reset();
                    currentStage = stage._160_pickup3;
                }
                break;
            case _160_pickup3:
                if(!follower.isBusy()) {
                    follower.followPath(grabPickup3a, powerNormal, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup3aPose;
                    currentStage = stage._170_pickup3_startintake;
                }
            break;

                case _170_pickup3_startintake:
                    if (!follower.isBusy()) {
                        currentTargetPose = pickup3bPose;
                        robot.intake.cmdFoward();
                        currentStage = stage._180_pickupa3;
                    }
                break;

            case _180_pickupa3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3b ,powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose= pickup3bPose;
                    pathTimer.reset();
                    currentStage = stage._190_ToScorePoseAP2;
                }
            break;

                case _190_ToScorePoseAP2:
                if (!follower.isBusy() || pathTimer.milliseconds() >= 2500) {
                    follower.followPath(scorePickup3,powerNormal,true);
                    currentTargetPose = scorePoseAP;
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._200_chkDrive_to_scorePoseAP2;
                }
             break;

            case _200_chkDrive_to_scorePoseAP2:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    runtime.reset();
                    currentStage = stage._210_ScorePickup3; // we don't need to do the turn since heading is adjusted in path

                }
            break;

            case _210_ScorePickup3:
                if (!follower.isBusy()) {
                   //
                    //
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("waiting to shoot 4");
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        runtime.reset();
                        currentStage = stage._450_Park;
                    }
                }

            case _450_Park:
                if (runtime.milliseconds() >= 1400) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    follower.followPath(endPath, powerFast,true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup2bPose;
                    currentStage = stage._500_End;
                }
                break;
            case _500_End:
            { //do nothing let the time run out

            }


            break;
        }

        updateTelemetry();
    }  //  loop

    private void updateTelemetry() {
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryMU.addData("LAST Pose", lastPose);
        telemetryMU.addData("Current Target Pose", currentTargetPose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P );
        telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D );
        telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F );
        telemetryMU.addData("CONSTRAINTS", "");
        telemetryMU.addData("Tvalue (% complete)", follower.pathConstraints.getTValueConstraint());
        telemetryMU.addData("Current tValue", follower.getCurrentTValue());
        telemetryMU.addData("Velocity Constraint", follower.pathConstraints.getVelocityConstraint());
        telemetryMU.addData("Current Velocity", follower.getVelocity());
        telemetryMU.addData("Trans constraint", follower.pathConstraints.getTranslationalConstraint());
        // telemetryMU.addData("current Trans", follower.getTranslationalError());
        telemetryMU.addData("Heading Constraint", follower.pathConstraints.getHeadingConstraint());

        telemetryMU.update();
        Drawing.drawDebug(follower);
    }

    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _20_DriveToScore,
        _25_checkDrivetoscore,
        _30_Shoot1,
        _40_LauncherStop,
        _50_Pickup1,
        _55_Pickup1_Startintake,
        _60_Pickup1a,
        _70_ToScorePoseAP,
        _75_chkDrive_to_score_P1,
        _80_ScorePickup1,
        _90_LauncherStop,
        _100_Pickup2,
        _110_Pickup2_Startintake,
        _120_Pickupa2,
        _130_ToScorePoseAP,
        _140_chkDrive_to_scorePoseAP,
        _150_ScorePickup2,
        _155_LauncherStop,
        _160_pickup3,
        _170_pickup3_startintake,
        _180_pickupa3,
        _190_ToScorePoseAP2,
        _200_chkDrive_to_scorePoseAP2,
        _210_ScorePickup3,
        _450_Park,
        _500_End


    }

}


