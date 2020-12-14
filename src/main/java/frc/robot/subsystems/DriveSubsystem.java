/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Encoder;


public class DriveSubsystem extends SubsystemBase {

    int frameNumber = 0 ;


    private Spark left1; // = new VictorSP(RobotMap.PWM.FRONT_LEFT_MOTOR_PORT);
    private Spark left2; // = new VictorSP(RobotMap.PWM.FRONT_RIGHT_MOTOR_PORT);
    private Spark right1; // = new VictorSP(RobotMap.PWM.BACK_LEFT_MOTOR_PORT);
    private Spark right2; // = new VictorSP(RobotMap.PWM.BACK_RIGHT_MOTOR_PORT);

    private final SpeedControllerGroup leftMotors ;
    private final SpeedControllerGroup rightMotors ;

    private final DifferentialDrive differentialDrive ;


    // The left-side drive encoder
    private final Encoder leftEncoder ;
    private final Encoder rightEncoder ;



    /**
     * Creates a new ExampleSubsystem.
     */
    public DriveSubsystem() {
     
        left1 = new Spark(2);
        left2 = new Spark(3);
        right1 = new Spark(4);
        right1.setInverted(true);
        right2 = new Spark(5);
        right2.setInverted(true);

        leftMotors = new SpeedControllerGroup( left1, left2 ) ;
        rightMotors = new SpeedControllerGroup( right1, right2 ) ;

        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

        leftEncoder = new Encoder(Constants.LeftEncoderPorts[0], Constants.LeftEncoderPorts[1], Constants.LeftEncoderReversed);
        leftEncoder.setDistancePerPulse( Constants.wheelDistancePerRevolution /Constants.countsPerRevolution );
        leftEncoder.reset();

        rightEncoder = new Encoder(Constants.RightEncoderPorts[0], Constants.RightEncoderPorts[1], Constants.RightEncoderReversed);
        rightEncoder.setDistancePerPulse( Constants.wheelDistancePerRevolution /Constants.countsPerRevolution );    
        rightEncoder.reset();
    }



    public void curvatureDrive(double speed, double rotation, boolean quickTurn){
      differentialDrive.curvatureDrive(speed, rotation, quickTurn);
//        System.out.format("DriveSybsystem.arcadeDrive(%f, %f)\n", speed, rotation) ;
  }



    public void arcadeDrive(double speed, double rotation){
        differentialDrive.arcadeDrive(speed, rotation);
//        System.out.format("DriveSybsystem.arcadeDrive(%f, %f)\n", speed, rotation) ;
    }


      /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }





    @Override
    public void periodic() {
    }
}
