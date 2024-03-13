// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.swerve.rev.RevSwerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;


public class IntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Flywheels m_Flywheels;
  private final Rollers m_Rollers;
  private final RevSwerve m_Drive;
  private Double NoteGrab = 0.2;
  private final PIDController noteAlignLR = new PIDController(1.2, 0.7, 0.1);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(Flywheels subsystem2, RevSwerve subsystem1, Rollers subsystem3) {
    m_Drive = subsystem1;
    m_Flywheels = subsystem2;
    m_Rollers = subsystem3;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Flywheels);
    addRequirements(m_Rollers);
    addRequirements(m_Drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", 1);
    m_Flywheels.IntakeCommand();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_Flywheels.hasNote.getAsBoolean()){
      NoteGrab = 0.0;
      m_Rollers.StopDouble();
    }
    else{
      NoteGrab = 0.2;
      m_Rollers.IntakeCommand();
    }

    new TeleopSwerve(m_Drive, 
      ()-> -NoteGrab, 
      ()-> -noteAlignLR.calculate(LimelightHelpers.getTX("limelight"), 0) * -0.012, 
      ()->0.0, 
      ()->true);

  /* While no note- drive forward, run flywheels. 
   * While note- run flywheels, run rollers, arm to float
   */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
