/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Solenoid;



public class IntakeSubsystem extends SubsystemBase {


  private Solenoid openSolenoid;
  private Solenoid closeSolenoid;


  /**
   * Creates a new ExampleSubsystem.
   */
  public IntakeSubsystem() {
    openSolenoid = new Solenoid(Constants.intakeDeployPort);
    closeSolenoid = new Solenoid(Constants.intakeRetractPort);
  }



  public void deploy() {
    openSolenoid.set( true );
    closeSolenoid.set( false ) ;
}



public void retract() {
    openSolenoid.set( false );
    closeSolenoid.set( true ) ;
}







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
