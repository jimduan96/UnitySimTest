/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;



/**
 * An example command that uses an example subsystem.
 */
public class DefaultDriveCommand extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;

  private DoubleSupplier forwardSupplier ;
  private DoubleSupplier steeringSupplier ;



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultDriveCommand(
      DriveSubsystem subsystem,
      DoubleSupplier f_supplier ,
      DoubleSupplier s_supplier 
      ) {
        driveSubsystem = subsystem;
        forwardSupplier = f_supplier ;
        steeringSupplier = s_supplier ;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      driveSubsystem.arcadeDrive(forwardSupplier.getAsDouble(), steeringSupplier.getAsDouble());
      //driveSubsystem.curvatureDrive(forwardSupplier.getAsDouble(), steeringSupplier.getAsDouble(), false);
  }

}
