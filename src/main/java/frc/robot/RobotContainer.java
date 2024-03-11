package frc.robot;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.swerve.rev.RevSwerve;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Flywheels;





/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    
    private final JoystickButton intakePOS = new JoystickButton(operator, 1);
    private final JoystickButton runFlywheel = new JoystickButton(operator, 4);
    private final JoystickButton ampScore = new JoystickButton(operator, 2);
    private final JoystickButton intake = new JoystickButton(operator, 5);


    private final JoystickButton temp1 = new JoystickButton(driver, 1);
    private final JoystickButton noteAlign = new JoystickButton(driver, 2);
    private final JoystickButton speakerAlign = new JoystickButton(driver, 3);
    private final JoystickButton pathToAmp = new JoystickButton(driver, 4);

    private final POVButton povUp = new POVButton(operator, 0);
    private final POVButton povDown = new POVButton(operator, 180);

    /* Subsystems */
    private final RevSwerve s_Swerve = new RevSwerve();
    private final Flywheels s_Flywheels = new Flywheels();
    private final Rollers s_Rollers = new Rollers();
    private final Arm s_Arm = new Arm();
    private final Climb s_Climb = new Climb();

    /* PIDs */
    private final PIDController speakerAlignLR = new PIDController(1.0, 0.0, 0.03);
    private final PIDController noteAlignLR = new PIDController(1.2, 0.7, 0.1);
    private double driveForwardVal = -0.2;

    /* Autonomous */
   // private final SendableChooser<Command> autoChooser;



  

  



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
          s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> driver.getRawAxis(translationAxis) * 0.3, 
                () -> driver.getRawAxis(strafeAxis) * 0.3, 
                () -> driver.getRawAxis(2) * 0.3, 
                () -> false
            )
        ); 

         s_Flywheels.setDefaultCommand(s_Flywheels.StopFlywheels());
         s_Rollers.setDefaultCommand(s_Rollers.StopDouble()); 
         s_Climb.setDefaultCommand(s_Climb.StopClimb());
         s_Arm.setDefaultCommand(s_Arm.armFloatingCommand());
         
         speakerAlignLR.setTolerance(0.5);
         noteAlignLR.setTolerance(0.1);
        // Configure the button bindings
        configureButtonBindings();

/* 

        NamedCommands.registerCommand("scoreSpeaker", s_Flywheels.RunFlywheels()
            .alongWith(new WaitCommand(1)).andThen(s_Rollers.Shoot())
            .alongWith(new WaitCommand(0.7).andThen(s_Flywheels.StopFlywheels()).alongWith(s_Rollers.StopDouble())));
        NamedCommands.registerCommand("intake", s_Flywheels.IntakeCommand()
            .alongWith(s_Rollers.IntakeCommand())
            .alongWith(new WaitCommand(0.7).andThen(s_Flywheels.StopFlywheels()).alongWith(s_Rollers.StopDouble())));
        NamedCommands.registerCommand("raiseArmToAmp", s_Arm.armToAmpCommand()
            .alongWith(new WaitCommand(2).andThen(s_Arm.StopArm())));
        NamedCommands.registerCommand("scoreAmp", s_Rollers.IntakeCommand()
            .alongWith(new WaitCommand(1).andThen(s_Rollers.StopDouble())));
        NamedCommands.registerCommand("armToIntakePose", s_Arm.armToIntakeCommand1()
            .alongWith(new WaitCommand(0.8).andThen(s_Arm.StopArm())));
        NamedCommands.registerCommand("raiseArmToSpeaker", s_Arm.armToSpeakerCommand()
            .alongWith(new WaitCommand(1).andThen(s_Arm.StopArm())));
        NamedCommands.registerCommand("armFloat", s_Arm.armFloatingCommand());

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser); 
        
        */


    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        /* Operator Buttons */

        runFlywheel.whileTrue(new ParallelCommandGroup((s_Arm.armToSpeakerCommand()).alongWith(s_Flywheels.RunFlywheels())));
        /* Shoots the note and then stops all things */
        runFlywheel.onFalse(new ParallelCommandGroup((s_Arm.armToSpeakerCommand()).alongWith(s_Flywheels.RunFlywheels().alongWith(s_Rollers.Shoot()
            .alongWith(new WaitCommand(0.7).andThen(s_Rollers.StopDouble().alongWith(s_Flywheels.StopFlywheels().alongWith(s_Arm.StopArm()))))))));

        ampScore.whileTrue(s_Arm.armToAmpCommand());
        ampScore.onFalse(new ParallelCommandGroup(s_Arm.armToAmpCommand().alongWith(s_Rollers.IntakeCommand()
            .alongWith(new WaitCommand(1).andThen(s_Rollers.StopDouble().alongWith(s_Arm.StopArm()))))));

        povUp.whileTrue(s_Climb.Extend());
        povDown.whileTrue(s_Climb.Retract());


        intakePOS.whileTrue(new SequentialCommandGroup(s_Arm.armToIntakeCommand1()
            .alongWith(new WaitCommand(1).andThen(s_Arm.StopArm()))));
        intakePOS.onFalse(s_Arm.StopArm()); 

     /*    speakerAlign.whileTrue(new TeleopSwerve(s_Swerve, 
        () -> driver.getRawAxis(translationAxis), 
        () -> driver.getRawAxis(strafeAxis), 
        () -> speakerAlignLR.calculate(LimelightHelpers.getTX("limelight"), 0) * -0.01, 
        () -> false
        )); */

        if(noteAlign.getAsBoolean()){
            LimelightHelpers.setPipelineIndex("limelight", 1);}
        else{
            LimelightHelpers.setPipelineIndex("limelilght", 0);}
    
            
        if(s_Flywheels.getSensor()){
                driveForwardVal = -0.2;
            }
        else if(!s_Flywheels.getSensor()) {
            driveForwardVal = 0.0;
        }   

    noteAlign.whileTrue(new TeleopSwerve(s_Swerve, 
        () -> driveForwardVal, 
        () -> driver.getRawAxis(strafeAxis), 
        () -> noteAlignLR.calculate(LimelightHelpers.getTX("limelight"), 0) * -0.012, 
        () -> true
        )
    );
/* 
    pathToAmp.whileTrue(AutoBuilder.pathfindToPose(
        new Pose2d(1.80, 7.60, Rotation2d.fromDegrees(-90)), 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ), 
        0, 
        2.0
      ));

    speakerAlign.whileTrue(AutoBuilder.pathfindToPose(
        new Pose2d(1.45, 5.60, Rotation2d.fromDegrees(180)), 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ), 
        0, 
        2.0
      )); */

        noteAlign.whileTrue(s_Flywheels.IntakeCommand());
       // intake.and(s_Flywheels.hasNote).whileTrue(s_Rollers.IntakeCommand());
       
        noteAlign.and(
            s_Flywheels.hasNote.whileFalse(s_Rollers.IntakeCommand().alongWith(s_Arm.armFloatingCommand()))
        );

    }

   //public class DriveSubsystem extends SubsystemBase {
  //public DriveSubsystem() {
    // All other subsystem initialization
    // ...

    // Configure AutoBuilder last
    
  //}
//}



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

/*     public Command getAutonomousCommand() {
        return autoChooser.getSelected();
      } */
    
}