/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.AnalogInput ;

import frc.robot.subsystems.DriveSubsystem;


public class DriveUntilClose extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_distance;
  private final double m_speed;
  private final AnalogInput distanceSensor ;

  /**
   * Creates a new DriveDistance.
   *
   * @param inches How close the robot willl get to the obstacle
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public DriveUntilClose(double inches, double speed, DriveSubsystem drive, AnalogInput distanceSensor) {
    m_distance = inches;
    m_speed = speed;
    m_drive = drive;
    this.distanceSensor = distanceSensor ;

    //addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.resetEncoders();
    m_drive.arcadeDrive(m_speed, 0);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.arcadeDrive(m_speed, 0);
  }


  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  @Override
  public boolean isFinished() {
    double distance = (5* distanceSensor.getVoltage() / (5.0 / 1024.0) ) * 0.0254 ; 
    return distance < m_distance ;
  }

}
