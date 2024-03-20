// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.rev.RevSwerve;

/** An example command that uses an example subsystem. */
public class DriveDistance extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

private  PIDController driveDistanceController = new PIDController(2.0, 0, 0);
private final RevSwerve m_swerve;
private Double m_distance;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveDistance(Double distance, RevSwerve Swerve) {
    driveDistanceController.setTolerance(0.02);
    m_swerve = Swerve;
    m_distance = distance;
    addRequirements(m_swerve);
  }
    // Use addRequirements() here to declare subsystem dependencies.


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetDriveEncoders();
        new TeleopSwerve(
                m_swerve, 
                () -> m_distance,
                () -> 0, 
                () -> 0, 
                () -> true
            );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new TeleopSwerve(
                m_swerve, 
                () -> 0,
                () -> 0, 
                () -> 0, 
                () -> true
            );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveDistanceController.atSetpoint();
  }
}
