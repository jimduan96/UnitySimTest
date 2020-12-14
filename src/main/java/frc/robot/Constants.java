package frc.robot; 


public class Constants{

    public static final double wheelDiameter = 8.0 ;
    public static final double wheelDistancePerRevolution = wheelDiameter * Math.PI;

    // PWM Ports
    public static final int LeftMotor1Port = 2;
    public static final int LeftMotor2Port = 3;
    public static final int RightMotor1Port = 4;
    public static final int RightMotor2Port = 5;

    // Encoder Ports
    public static final int[] LeftEncoderPorts = new int[]{0, 1};
    public static final int[] RightEncoderPorts = new int[]{2, 3};
    public static final boolean LeftEncoderReversed = false;
    public static final boolean RightEncoderReversed = true;
    public static final int countsPerRevolution = 4096 ;

    //General
    public static final double degreesToRadiansFactor = Math.PI/180;

    //Wheel
    public static final int armExtendSolenoidPort = 1;
    public static final int armRetractSolenoidPort = 0;

    // Intake
    public static final int intakeDeployPort = 2;
    public static final int intakeRetractPort = 3;


    //OI
    public static final int joystickPort = 0;
    public static final int boxControlPort = 1;

    public static final int armExtendButton = 1 ;


}