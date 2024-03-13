// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Rollers;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class ScoreAmp extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_Arm;
  private final Rollers m_Rollers;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoreAmp(Arm subsystem1, Rollers subsystem3) {
    m_Arm = subsystem1;
    m_Rollers = subsystem3;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);
    addRequirements(m_Rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.armToAmpCommand();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Rollers.ScoreAmp().alongWith(new WaitCommand(0.5).andThen(m_Arm.StopArm().alongWith(m_Rollers.StopDouble())));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
