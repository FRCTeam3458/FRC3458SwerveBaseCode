// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //public OuterIntake() {}
   private final Victor doubleRoller = new Victor(16);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command IntakeCommand() {
    return runOnce(() -> doubleRoller.set(0.5))
          .withName("Intake"); 
  }

  public Command ShootCommand() {
    return runOnce(() -> doubleRoller.set(-0.5))
            .withName("Shoot");
  }

  public Command alignNote() {
    return runOnce(() -> doubleRoller.set(0.5))
            .withName("Align Note");
  }

  public Command StopRollersCommand() {
    return runOnce(() -> doubleRoller.set(0))
            .withName("Stop Double Rollers");
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("Intake command", IntakeCommand());
    SmartDashboard.putData("StopRollers", StopRollersCommand());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
