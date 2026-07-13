package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Mat;
import org.opencv.core.RotatedRect;

public class DeepRoboConstants {

    //region - Hardware
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;

    DcMotor lift1;
    DcMotor lift2;
    DcMotor lift3;

    CRServo testservo1;
    CRServo testservo2;

    Servo arml;
    Servo armr;
    Servo claw;
    Servo clawrotate;
    Servo inExtL;
    Servo inExtR;
    Servo intakepivot;
    Servo intakeclaw;
    Servo intakewrist;

    AnalogInput Fus;
    AnalogInput Bus;
    AnalogInput Rus;
    AnalogInput Lus;

    AnalogInput testservo1encoder;
    AnalogInput testservo2encoder;


    DigitalChannel liftMag;
    DigitalChannel clawbeam;

    DigitalChannel pin0;
    DigitalChannel pin1;

    SparkFunOTOS otos;
    GoBildaPinpointDriver odo;
    //endregion

    //region - Drive Variables
    double flpower;
    double frpower;
    double blpower;
    double brpower;

    double power = 1;
    double halfpower = 0.5;
    double fullpower = 1;

    double z_min = 0.1; //was 0.1
    double z_tolerance = 0.02;
    double ztarget = 0;
    double zfastmin = 0.2;

    double y_min = 0.1; //0.075
    double y_tolerance = 0.01;
    double ytarget = 0;
    double yfastmin = 0.3;

    double x_min = 0.15; //0.125
    double x_tolerance = 0.01;
    double xtarget = 0;
    double xfastmin = 0.3;

    double yerror = 0;
    double xerror = 0;
    double zerror = 0;

    double ypower = 0;
    double xpower = 0;
    double zpower = 0;

    double headingdiff = 0;

    double xp = 0;
    double yp = 0;

    boolean fastloopy = true;
    boolean fastloopx = true;

    double bartolerance = -4;

    //double LusDist = 0.1383 + (0.0115*(Lus.getVoltage()));
    //double BusDist = 0.1378 + (0.01122*(Bus.getVoltage()));
    //double RusDist = 0.1377 + (0.01127*(Rus.getVoltage()));
    //endregion

    //region - Intake Variables
    double intakerightin = 0.03; // 0 //0.06
    double intakerightout = 0.54; //.59
    double intakeleftin = .84; // 1 //.92 //0.91
    double intakeleftout = 0.36; //.48 //0.44

    double intakepivotdown = 0.79; //0.77
    double intakepivotmid = 0.55;
    double intakepivotup = 0.2; //43
    double intakepivotback = 0.16; //41
    double samplescan = 0.63; //85 //check this and maybe adjust for auto to read better?
    double intakepivotscan = 0.12;

    double intakeopen = 0.58; //0.65
    double intakeclosed = 1;

    double intakewrist0 = 0.59; //start position
    double intakewrist25 = 0.68;
    double intakewrist45 = 0.77;
    double intakewrist90 = 0.93;
    double intakewrist_45 = 0.43;
    double intakewrist_90 = 0.25;
    //endregion

    //region - Arm, Claw, Lift Variables
    double clawopenfull = 0.38;
    double clawautograb = 0.23; //0.3
    double clawclosed = 0.1; //0.09

    double clawrotatetransfer = 0.63;
    double clawflat = 0.33;
    double clawrotatebackgrab = 0.53; //56
    double clawrotatedeliverb = 0.24;
    double clawrotatedown = 0.57;
    double clawrotateup = 0.13;
    double clawrotatebar2 = 0.28; //0.3
    double clawrotatescan = 0.52; //
    double clawrotateinit = 0.04;
    double clawrotatehang = 0.55;//.538 //.546//.575

    double clawrotatescan2 = 0.52; //created due to faulty camera code in presentation program


    double armltransfer = 0.25; //0.23
    double armrtransfer = 0.78; //0.8
    double armlback1 = 0.86;
    double armrback1 = 0.18;
    double armlbackgrab = 0.9; //94
    double armrbackgrab = 0.1; //1
    double armlup = 0.44;
    double armrup = 0.58;
    double armlpark = 0.32; //0.28
    double armrpark = 0.7; //0.74
    double armldeliver = 0.51;
    double armrdeliver = 0.52;
    double armlbarpt1 = 0.29; //same as park?
    double armrbarpt1 = 0.72; //same as park?
    double armlbarpt2 = 0.42; //0.37
    double armrbarpt2 = 0.59; //0.64
    double armlmove = 0.31;
    double armrmove = 0.71;
    double armlinit = 0.07;
    double armrinit = 0.95;
    double armlhang = 0.02; //.102
    double armrhang = 0.97; //.875
    double armlspecdrop = 0.67; //0.7
    double armrspecdrop = 0.35; //0.32

    boolean liftreset = false;
    boolean loopcheck = true;

    int liftdown = 0;
    int lifthigh = 2060; //2150
    int Lbar2 = 400;
    int Lgrab = 130; //120
    int liftmax = 2280;
    int liftholdmin = 50;
    int lifthanglim = 600;
    int liftlow = 600;
    int liftgap = 300;

    double liftholdpower = 0.06; //0.08
    double liftlowpower = 0.3;
    double liftbarpower = 1; //0.6
    double liftdowntimelimit = 5000;
    //endregion

    //region - Test Drive Positions
    Pose2D odoorigin = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    SparkFunOTOS.Pose2D drivetarget;
    SparkFunOTOS.Pose2D roboorigin = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D drive0 = new SparkFunOTOS.Pose2D(0, 20, 0);
    SparkFunOTOS.Pose2D drive45 = new SparkFunOTOS.Pose2D(20, 20, -45);
    SparkFunOTOS.Pose2D drive90 = new SparkFunOTOS.Pose2D(20, 0, -90); //was -90 but -45 for translation
    SparkFunOTOS.Pose2D drive135 = new SparkFunOTOS.Pose2D(20, -20, -135);
    SparkFunOTOS.Pose2D drive180 = new SparkFunOTOS.Pose2D(0, -20, 180);
    SparkFunOTOS.Pose2D drive225 = new SparkFunOTOS.Pose2D(-20, -20, 135);
    SparkFunOTOS.Pose2D drive270 = new SparkFunOTOS.Pose2D(-20, 0, 90); //was 90 but 45 for translation
    SparkFunOTOS.Pose2D drive315 = new SparkFunOTOS.Pose2D(-20, 20, 45);
    //endregion

    //region - OTOS & Odo
    SparkFunOTOS.Pose2D location = roboorigin;
    SparkFunOTOS.Pose2D otosloc;
    Pose2D odopos;

    double xodooffset = -36;
    double yodooffset = -19.1;
    //endregion

    //region - Menu Variables
    String startpos = "Left";
    String teamcolor = "Red";
    String GrabColor = "Yellow";
    //endregion

    //region - Camera Variables
    String colorseen = "None";

    int w = 320;
    int h = 240;
    double topborder = 0.64;

    double lineanles = 0;

    ColorBlobLocatorProcessor colorLocator;
    VisionPortal portal;

    double camrot = 0;
    double contourbottom = 0;
    double contourinneredge = 0;

    double sampledist = 6.5;
    double disttoservo = 0.12;

    double L0camlim = 127;
    double L90camlim = 130; //changed based on equation
    double R0camlim = 182;
    double R90camlim = 178; //changed based on equation

    double offdist = 0;
    double offpixels = 0;
    double pixtoinR = 33;
    double pixtoinL = 30;
    double pixeltoindistR = 33;
    double pixeltoindistL = 30;
    double extradist = 0;
    double distdiff = 0;

    boolean needtostrafe = false;

    //region - OLD
    double Lcamlim = 92;
    double Rcamlim = 204;
    //endregion

    RotatedRect box;
    ColorBlobLocatorProcessor.Blob BLOB;
    double blobcount = 0;
    double yellowresetcount = 0;

    String colortodrop;

    ColorRange colortarget = ColorRange.YELLOW;
    SparkFunOTOS.Pose2D pixelpos = new SparkFunOTOS.Pose2D(0, 0, 0);
    //endregion

    //region - Left
    //region - Left Drive Positions - Meet 2
    SparkFunOTOS.Pose2D leftdrivebucket = new SparkFunOTOS.Pose2D(-18, 6, -45); //-6, -18, 45
    SparkFunOTOS.Pose2D leftdrivebucket2 = new SparkFunOTOS.Pose2D(-18, 1, -45); //-3, -21, 45
    SparkFunOTOS.Pose2D leftgrabL = new SparkFunOTOS.Pose2D(-24, 13.5, 24); //5, 15, 115 for ultrasonic, 5, 15, 115
    SparkFunOTOS.Pose2D leftgrabM = new SparkFunOTOS.Pose2D(-22, 13, 0); //5, 15, 90 for ultrasonic, -25, -23, 75
    SparkFunOTOS.Pose2D leftgrabR = new SparkFunOTOS.Pose2D(-12, 12.5, 0); //14.5, 15, 90 for ultrasonic, -23, -20.5, 51
    SparkFunOTOS.Pose2D leftparkpt1 = new SparkFunOTOS.Pose2D(-8, 26, 0); //-26, 0, 0
    SparkFunOTOS.Pose2D leftparkpt2 = new SparkFunOTOS.Pose2D(-10, 42, 0); //-52, 0, 0
    SparkFunOTOS.Pose2D leftparkpt3 = new SparkFunOTOS.Pose2D(20, 50, -85); //-52, 15, 0 //53
    //endregion

    //region - Left Drive Positions - Meet 3
    SparkFunOTOS.Pose2D Lbucket = new SparkFunOTOS.Pose2D(-12.5, -8, 0);
    SparkFunOTOS.Pose2D LgrabL = new SparkFunOTOS.Pose2D(-23.5, 3.25, 72); //-20.5, 0.5
    SparkFunOTOS.Pose2D LgrabM = new SparkFunOTOS.Pose2D(-21.5, 1, 45); //-18, -1.5
    SparkFunOTOS.Pose2D LgrabR = new SparkFunOTOS.Pose2D(-16.5, 6.5, 45); //-11.5, 4
    SparkFunOTOS.Pose2D Lpark2 = new SparkFunOTOS.Pose2D(-55, 38, 0);
    SparkFunOTOS.Pose2D Lpark3 = new SparkFunOTOS.Pose2D(-35, 60, -48); //-30, 56
    SparkFunOTOS.Pose2D Lgrab2 = new SparkFunOTOS.Pose2D(-26, 63, -45);
    SparkFunOTOS.Pose2D Lgrab3 = new SparkFunOTOS.Pose2D(-30, 70, -45);
    SparkFunOTOS.Pose2D Lgrab4 = new SparkFunOTOS.Pose2D(-33, 77, -45);
    SparkFunOTOS.Pose2D Lpostgrab = new SparkFunOTOS.Pose2D(-33, 32, 0);
    SparkFunOTOS.Pose2D leftgrab = new SparkFunOTOS.Pose2D(0, 0, 0);
    //endregion

    //region - Left Drive Pos - Champs
    SparkFunOTOS.Pose2D deliver1 = new SparkFunOTOS.Pose2D(-17, 5, -45); //-9, 11 (odo) after -18, 6, -45
    SparkFunOTOS.Pose2D deliver0 = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D floorgrabR = new SparkFunOTOS.Pose2D(-7, 18, 0); //-6.5, 18.5 //-3, 13, 45 //-7, 9.5 (odo) after -4, 14.5, after -20.5, 0.5
    SparkFunOTOS.Pose2D floorgrabM = new SparkFunOTOS.Pose2D(-16, 18.5, 0); //-10, 10, 45 //-1, 16.5 (odo) after -9, 9, after -18, -1.5
    SparkFunOTOS.Pose2D floorgrabL = new SparkFunOTOS.Pose2D(-18, 26, 48); //-18, 12, 75 //-2, 19 (odo) after -11, 11.25, after -11.5, 4
    SparkFunOTOS.Pose2D presub = new SparkFunOTOS.Pose2D(-12, 45, -25); //-23, 37, 0 //-19.5, 30 (odo) after -42.5, 46
    SparkFunOTOS.Pose2D park = new SparkFunOTOS.Pose2D(18, 47.5, -85); //-15, 58, -45 //0, 48 (odo) after -22.5, 68, before -30, 56
    SparkFunOTOS.Pose2D grab2 = new SparkFunOTOS.Pose2D(18, 74, -85); //54 //-18, 61, -45 //6, 52 (odo) after -13.5, 71
    SparkFunOTOS.Pose2D grab3 = new SparkFunOTOS.Pose2D(18, 60, -85); //-19, 62, -45 //-4, 44 (odo) after -17.5, 78
    SparkFunOTOS.Pose2D grab4 = new SparkFunOTOS.Pose2D(18, 67, -85); //-24.5, 67, -45 //-8, 38 (odo) after -20.5, 85
    SparkFunOTOS.Pose2D postsub = new SparkFunOTOS.Pose2D(-3, 15, 0); //-20.5, 40 (odo) after -20.5, 40
    SparkFunOTOS.Pose2D subgrab = new SparkFunOTOS.Pose2D(0, 0, 0);
    //endregion

    //region - Left Drive Pos - Reg
    SparkFunOTOS.Pose2D del0 = new SparkFunOTOS.Pose2D(-16, 6, -45); // y was 6//-9, 11 (odo) after -18, 6, -45
    SparkFunOTOS.Pose2D del = new SparkFunOTOS.Pose2D(-15, 6, -45); // y was 6//-9, 11 (odo) after -18, 6, -45
    SparkFunOTOS.Pose2D del2 = new SparkFunOTOS.Pose2D(-17.5, 4, -45); //y was 4//-9, 11 (odo) after -18, 6, -45
    SparkFunOTOS.Pose2D del3 = new SparkFunOTOS.Pose2D(-19, 4.25, -45);// y was 4.25 //-9, 11 (odo) after -18, 6, -45
    SparkFunOTOS.Pose2D fgr = new SparkFunOTOS.Pose2D(-22, 14.25, -25); //14 //y 13.75// y was 13.25 //13.75 //-22.5, 13 //-6.5, 18.5 //-3, 13, 45 //-7, 9.5 (odo) after -4, 14.5, after -20.5, 0.5
    SparkFunOTOS.Pose2D fgm = new SparkFunOTOS.Pose2D(-23, 14, 0); //14.25 //13.75 //y was 13.25//-22.75, //-10, 10, 45 //-1, 16.5 (odo) after -9, 9, after -18, -1.5
    SparkFunOTOS.Pose2D fgl = new SparkFunOTOS.Pose2D(-20.5, 18.5, 30); //18.25 //ywas 17.75 //current y 18.5 //-20.25 //18.75 //-19.75, 19 //-18, 12, 75 //-2, 19 (odo) after -11, 11.25, after -11.5, 4
    SparkFunOTOS.Pose2D tosub1 = new SparkFunOTOS.Pose2D(-12, 60, -20); //-8 //-23, 37, 0 //-19.5, 30 (odo) after -42.5, 46
    SparkFunOTOS.Pose2D tosub2 = new SparkFunOTOS.Pose2D(-10, 60, -20); //-8 //-23, 37, 0 //-19.5, 30 (odo) after -42.5, 46
    SparkFunOTOS.Pose2D tobucket = new SparkFunOTOS.Pose2D(3, 35, -20); //-23, 37, 0 //-19.5, 30 (odo) after -42.5, 46
    SparkFunOTOS.Pose2D PARK = new SparkFunOTOS.Pose2D(18, 55, -88); //-85 //57 //-15, 58, -45 //0, 48 (odo) after -22.5, 68, before -30, 56
    SparkFunOTOS.Pose2D END = new SparkFunOTOS.Pose2D(18, 60, -85); //57 //-15, 58, -45 //0, 48 (odo) after -22.5, 68, before -30, 56
    SparkFunOTOS.Pose2D GRAB = new SparkFunOTOS.Pose2D(18, 55, -85);
    //endregion
    //endregion

    //region - Right Drive Positions
    //region - OLD
    //region - Meet 2 Deliver
    SparkFunOTOS.Pose2D rightpredrivebar = new SparkFunOTOS.Pose2D(0, 4, 0); //25
    SparkFunOTOS.Pose2D rightdrivebar = new SparkFunOTOS.Pose2D(-17, 32, 0); //25
    SparkFunOTOS.Pose2D rightdrivebar2 = new SparkFunOTOS.Pose2D(-19, 33.25, 0); //25
    SparkFunOTOS.Pose2D rightdrivebar3 = new SparkFunOTOS.Pose2D(-12, 33.5, 0); //25
    SparkFunOTOS.Pose2D rightdrivebar4 = new SparkFunOTOS.Pose2D(-10, 33.5, 0); //25
    //endregion

    //region - Meet 3 Positions
    SparkFunOTOS.Pose2D Rprebar = new SparkFunOTOS.Pose2D(0, 4, 0); //25
    SparkFunOTOS.Pose2D Rbar = new SparkFunOTOS.Pose2D(17, -32, 0); //25
    SparkFunOTOS.Pose2D Rbar2 = new SparkFunOTOS.Pose2D(19, -33.25, 0); //25
    SparkFunOTOS.Pose2D Rbar3 = new SparkFunOTOS.Pose2D(12, -33.5, 0); //25
    SparkFunOTOS.Pose2D Rbar4 = new SparkFunOTOS.Pose2D(10, -33.5, 0); //25
    //endregion

    //region - Move All Then Score
    SparkFunOTOS.Pose2D Rgrab1 = new SparkFunOTOS.Pose2D(-27, -28, 0);
    SparkFunOTOS.Pose2D Rmove1 = new SparkFunOTOS.Pose2D(-27, -52, 0);
    SparkFunOTOS.Pose2D Rgrab2 = new SparkFunOTOS.Pose2D(-33, -52, 0);
    SparkFunOTOS.Pose2D Rmove2 = new SparkFunOTOS.Pose2D(-33, -7, 0);
    SparkFunOTOS.Pose2D Rgrab3 = new SparkFunOTOS.Pose2D(-42, -52, 0);
    SparkFunOTOS.Pose2D Rmove3 = new SparkFunOTOS.Pose2D(-42, -7, 0);

    SparkFunOTOS.Pose2D Rtrade1 = new SparkFunOTOS.Pose2D(40, 4, 0); //40.75, 0?
    SparkFunOTOS.Pose2D Rtrade2 = new SparkFunOTOS.Pose2D(40, 1, 0); //40.75, 0?
    //endregion

    //region - Meet 2
    SparkFunOTOS.Pose2D rightgrabL = new SparkFunOTOS.Pose2D(38.5, 14.5, 0); //25, 23.25, -42
    SparkFunOTOS.Pose2D rightLdrop = new SparkFunOTOS.Pose2D(24, 18, -137);
    SparkFunOTOS.Pose2D rightgrabM = new SparkFunOTOS.Pose2D(47, 14.5, 0); //34, 24.25, -42
    SparkFunOTOS.Pose2D rightMdrop = new SparkFunOTOS.Pose2D(35, 24.25, -144);
    SparkFunOTOS.Pose2D rightgrabR = new SparkFunOTOS.Pose2D(50, 15.5, -30); //44.75, 23.25, -40

    SparkFunOTOS.Pose2D righttrade = new SparkFunOTOS.Pose2D(40, 15, 180); //40.75, 0?
    SparkFunOTOS.Pose2D righttrade2 = new SparkFunOTOS.Pose2D(40, 3.5, 180); //0?
    SparkFunOTOS.Pose2D righttrade2final = new SparkFunOTOS.Pose2D(40, 4.5, 180); //0?
    SparkFunOTOS.Pose2D rightposttrade = new SparkFunOTOS.Pose2D(40, 3, 0);
    //endregion

    SparkFunOTOS.Pose2D rightpark = new SparkFunOTOS.Pose2D(45, 5, 0);
    SparkFunOTOS.Pose2D rdrive1 = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D rdrive2 = new SparkFunOTOS.Pose2D(0, 0, 0);
    SparkFunOTOS.Pose2D rdrive3 = new SparkFunOTOS.Pose2D(0, 0, 0);
    //endregion

    SparkFunOTOS.Pose2D barscore = new SparkFunOTOS.Pose2D(-6, 35.5, 0); //36
    SparkFunOTOS.Pose2D RgrabL = new SparkFunOTOS.Pose2D(41, 17.5, 0); //43, 20
    SparkFunOTOS.Pose2D RgrabM = new SparkFunOTOS.Pose2D(51, 18, 0); //53, 20
    SparkFunOTOS.Pose2D RgrabR = new SparkFunOTOS.Pose2D(52, 19.5, -25); //53, 22
    SparkFunOTOS.Pose2D Rtrade = new SparkFunOTOS.Pose2D(32, 8, 0);
    SparkFunOTOS.Pose2D Rgrab = new SparkFunOTOS.Pose2D(32, 0, 0);
    //endregion

    public void configureOtos() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // If sensor is 5 inches left (neg X) and 10 inches forward (pos y) of center
        // of robot, and 90 degrees clockwise (neg rotation) from robot's orientation,
        // offset is {-5, 10, -90}
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.1181, 0.8976, 180);
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otos.setLinearScalar(1.126); //0.9725
        otos.setAngularScalar(0.993);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        otos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        otos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);
    }

    public void configureOdo() {
        odo.setOffsets(xodooffset, yodooffset); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
    }
}
