/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AddressableLED;

import edu.wpi.first.wpilibj.DigitalOutput;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {


  private RobotContainer m_robotContainer;

  private Command m_autonomousCommand;
  private Command testCommand ;

  private AnalogInput ai ;


  private DigitalOutput digOut = new DigitalOutput(7) ;
  private boolean diSet= false ;

  int wpk = 0 ;
  



  public Robot() {

    ai = new AnalogInput(0) ;
    AddressableLED led = new AddressableLED(1) ;
//    PhysicsSim.getInstance().addTalonSRX(_talon, 0.75, 4000, true);

  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //System.out.println("joystick forward axis is " + controller.getRawAxis(1)) ;

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if ( diSet ) {
      digOut.set(true) ;
    } else {
      digOut.set(false) ;
    }
    diSet =!diSet ;


  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    System.out.println("Initialized disable") ;

    System.out.println("The aliance is " + DriverStation.getInstance().getAlliance()) ;
  }

  @Override
  public void disabledPeriodic() {
    wpk++;
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    System.out.println("Initialized autonomous") ;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    wpk++ ;
  }

  @Override
  public void teleopInit() {
    System.out.println("Initialized teleop") ;

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    wpk++;

  }

  @Override
  public void testInit() {
    System.out.println("Initialized test") ;

    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    testCommand = m_robotContainer.getTestCommand() ;
    testCommand.schedule();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    wpk++ ;
  }



  public void simulationInit() {
//    PhysicsSim.getInstance().addTalonSRX(_talon, 0.75, 7200, true);
  }

  public void simulationPeriodic() {
    PhysicsSim.getInstance().run();
  }



}
