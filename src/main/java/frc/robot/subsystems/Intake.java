// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //public OuterIntake() {}
   private final CANSparkMax bottomRoller = new CANSparkMax(8, MotorType.kBrushless);
   private final CANSparkMax upperRoller = new CANSparkMax(9, MotorType.kBrushless);
   private final VictorSP doubleRoller = new VictorSP(10);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command IntakeCommand() {
    return runOnce(() -> upperRoller.set(0.5))
          .andThen(run(() -> bottomRoller.set(-0.5)))
          .andThen(run(() -> doubleRoller.set(0.5)))
          .withName("Intake"); 
  }

  public Command Flywheel() {
    return runOnce(() -> upperRoller.set(-1))
            .andThen(run(() -> bottomRoller.set(1)))
            .withName("Flywheel");
  }
  public Command Shoot() {
    return runOnce(() -> doubleRoller.set(-0.5))
            .withName("Shoot");
  }
  public Command alignNote() {
    return runOnce(() -> doubleRoller.set(0.5))
            .withName("Align Note");
  }
  public Command StopDouble() {
    return runOnce(() -> doubleRoller.set(0))
            .withName("Stop Double Rollers");
  }
  public Command StopFlywheels() {
    return runOnce(() -> upperRoller.set(0.0))
            .andThen(() -> bottomRoller.set(0.0))
            .withName("Stop Flywheels");
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
