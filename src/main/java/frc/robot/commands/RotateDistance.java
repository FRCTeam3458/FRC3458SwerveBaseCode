// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.rev.RevSwerve;

/** An example command that uses an example subsystem. */
public class RotateDistance extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

private  PIDController rotDistanceController = new PIDController(2.0, 0, 0);
private final RevSwerve m_swerve;
private Double m_distance;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateDistance(Double distance, RevSwerve Swerve) {
    rotDistanceController.setTolerance(2);
    m_swerve = Swerve;
    m_distance = distance;
    addRequirements(m_swerve);

  }
    // Use addRequirements() here to declare subsystem dependencies.


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetDriveEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new TeleopSwerve(
                m_swerve, 
                () -> 0,
                () -> 0, 
                () -> (rotDistanceController.calculate(m_swerve.getYaw().getDegrees(), m_distance) * -0.1),
                () -> true
            );
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
    return rotDistanceController.atSetpoint();
  }
}
