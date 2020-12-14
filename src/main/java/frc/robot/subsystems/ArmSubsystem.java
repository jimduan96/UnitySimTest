/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;


public class ArmSubsystem extends SubsystemBase {

  private DoubleSolenoid theSolenoid ;
  private Spark wheelSpinner;



  
  /**
   * Creates a new ExampleSubsystem.
   */
  public ArmSubsystem() {

    theSolenoid = new DoubleSolenoid(Constants.armExtendSolenoidPort, Constants.armRetractSolenoidPort ) ;
    // do we need to add a requirement here?
  }



  public void extend() {
      theSolenoid.set( DoubleSolenoid.Value.kForward) ;
  }



  public void retract() {
      theSolenoid.set( DoubleSolenoid.Value.kReverse) ;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
