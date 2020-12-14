/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



import frc.robot.subsystems.DriveSubsystem;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveDistance;


import frc.robot.subsystems.ArmSubsystem;

import frc.robot.subsystems.IntakeSubsystem;

// import frc.robot.commands.ArmExtendCommand;
// import frc.robot.commands.ArmRetractCommand;


import edu.wpi.first.wpilibj.Joystick;



// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTableEntry;

// import frc.robot.OI;


// for testing of network tables based sim.
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;




/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private Joystick controller; 
  private OI operatorInterface = new OI() ;

  // Declarations for drive subsystem
  private DriveSubsystem driveSubsystem ;
  // A simple auto routine that drives forward a specified distance, and then stops.
  private Command simpleAuto ; 

  
  // Declarations for Arm subsystem
  private ArmSubsystem armSubsystem ;
  private JoystickButton armExtendButton ;
  // private ArmExtendCommand armExtend ;
  // private ArmRetractCommand armRetract ;


  // Declarations for Intake subsystem
  private IntakeSubsystem intake ;
  private JoystickButton intakeButton ;


  
  boolean armExtended = false ;

  boolean intakeOpen = false ;





  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

      // Configure the button bindings
      configureButtonBindings();

      controller = new Joystick(Constants.joystickPort);





      //--------------------------------------------
      // Set up the drive subsystem
      //--------------------------------------------
      driveSubsystem = new DriveSubsystem() ;
      driveSubsystem.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new DefaultDriveCommand(
          driveSubsystem,
            () -> -operatorInterface.getForwardSpeed(),
            () -> operatorInterface.getTurnAngle()
            )
        );

      simpleAuto = new DriveDistance(60, 0.5, driveSubsystem);

      //--------------------------------------------
      // Set up the arm subsystem
      //--------------------------------------------
      armSubsystem = new ArmSubsystem() ;
      armExtendButton = new JoystickButton(controller, 1);
      armExtendButton.whenPressed(() -> 
         { armExtended = !armExtended ;
          if (armExtended) {
              armSubsystem.extend();
          } else {
            armSubsystem.retract();
          }
         }
      ) ;


      //--------------------------------------------
      // Set up the intake subsystem
      //--------------------------------------------
      intake = new IntakeSubsystem() ;
      intakeButton = new JoystickButton(controller, 2);
      intakeButton.whenPressed(() -> 
         { intakeOpen = !intakeOpen ;
          if (intakeOpen) {
              intake.deploy();
          } else {
            intake.retract();
          }
         }
      ) ;


  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return simpleAuto ;
  } 
}
