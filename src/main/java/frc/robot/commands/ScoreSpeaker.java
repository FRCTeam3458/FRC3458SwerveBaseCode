// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Rollers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ScoreSpeaker extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_Arm;
  private final Flywheels m_Flywheels;
  private final Rollers m_Rollers;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoreSpeaker(Arm subsystem1, Flywheels subsystem2, Rollers subsystem3) {
    m_Arm = subsystem1;
    m_Flywheels = subsystem2;
    m_Rollers = subsystem3;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);
    addRequirements(m_Flywheels);
    addRequirements(m_Rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Flywheels.RunFlywheels();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.armToSpeakerCommand();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Rollers.Shoot().alongWith(new WaitCommand(0.7).andThen(m_Flywheels.StopFlywheels()));
    m_Arm.StopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
