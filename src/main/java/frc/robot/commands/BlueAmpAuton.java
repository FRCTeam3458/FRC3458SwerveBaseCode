// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.swerve.rev.RevSwerve;

/** An example command that uses an example subsystem. */
public class BlueAmpAuton extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

private  PIDController driveDistanceController = new PIDController(2.0, 0, 0);
private final RevSwerve m_swerve;
private final Rollers m_rollers;
private final Flywheels m_flywheels;
private final Arm m_arm;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BlueAmpAuton(RevSwerve swerve, Rollers rollers, Flywheels flywheels, Arm arm) {
    driveDistanceController.setTolerance(0.02);
    m_swerve = swerve;
    m_rollers = rollers;
    m_flywheels = flywheels;
    m_arm = arm;
    addRequirements(m_swerve, m_flywheels, m_rollers, m_arm);
  
    addCommands(
      m_arm.armToAmpCommandAuto().alongWith(new StrafeDistance(0.38, m_swerve)), m_arm.stopArmAuto(), new DriveDistance(0.43, m_swerve), 
      m_rollers.ScoreAmpAuto(), m_rollers.StopDoubleAuto(), m_arm.armFloatingCommandAuto(), new DriveDistance(-1.19, m_swerve), new RotateDistance(0.0, m_swerve),
      m_arm.armToIntakeCommandAuto(), 
      new ParallelRaceGroup(m_rollers.IntakeCommand().alongWith(m_flywheels.IntakeCommand()).alongWith(new DriveDistance(1.0, m_swerve))), 
      m_rollers.StopDoubleAuto(), m_flywheels.StopFlywheelsAuto(),
      new DriveDistance(-1.0, m_swerve), new RotateDistance(270.0, m_swerve), m_arm.armToAmpCommandAuto(), m_arm.stopArmAuto(),
      new DriveDistance(1.19, m_swerve), m_rollers.ScoreAmpAuto(), m_rollers.StopDouble()

    );


  }
}
