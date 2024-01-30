// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Arm() {}

  private final VictorSP armMotor = new VictorSP(12);

  private final Encoder armEncoder = new Encoder(1, 2);
  private final PIDController armController = new PIDController(1.0, 0.0, 0,0);
  private final ArmFeedforward armForward = new ArmFeedforward(0, 0, 0);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command armToAmpCommand() {
    return runOnce(() -> 
      armMotor.set(armController.calculate(armEncoder.getDistance(), 20) + armForward.calculate(20, 0)))
          .withName("Arm to Amp");
        }
    
  public Command armToSpeakerCommand() {
      return runOnce(() -> 
        armMotor.set(armController.calculate(armEncoder.getDistance(), 40) + armForward.calculate(40, 0)))
            .withName("Arm to Speaker");
        }

  public Command armToPickupCommand() {
        return runOnce(() -> 
          armMotor.set(armController.calculate(armEncoder.getDistance(), 80) + armForward.calculate(80, 0)))
              .withName("Arm to Pickup");
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
