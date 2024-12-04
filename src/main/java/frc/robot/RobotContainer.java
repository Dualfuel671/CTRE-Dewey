// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller; // We added this for the game controllers we have.
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController; (this came with the project template)
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Subsystems.Arm;
//
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Wrist;
//import frc.robot.Subsystems.TestFolder.AmperageTest;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  //private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final PS4Controller driver = new PS4Controller(0); // PS4 controller
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Wrist wrist = new Wrist();
  private final Arm arm = new Arm();
  //private final Intake intake = new Intake();
  // private final AmperageTest intake = new AmperageTest();
  private final Shooter shooter = new Shooter();
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final JoystickButton crossButton = new JoystickButton(driver, 2);
  private final JoystickButton squareButton = new JoystickButton(driver, 1);
  private final JoystickButton leftBumper = new JoystickButton(driver, 5);
  private final JoystickButton rightBumper = new JoystickButton(driver, 6);
  

  private final JoystickButton leftTriggerButton = new JoystickButton(driver, 7);
  private final JoystickButton rightTriggerButton = new JoystickButton(driver,8 );
  //private final JoystickButton someButtonA = new JoystickButton(driver, 0);
 // private final JoystickButton someButtonB = new JoystickButton(driver, 0);
  
  private final JoystickButton triangle = new JoystickButton(driver, 4);
  private final JoystickButton circle = new JoystickButton(driver, 3);
  private final JoystickButton touchpad= new JoystickButton(driver, 14);
  // Check Drivers station to determine what the POV buttons are...connect controller and toggle buttons on "usb"
  private final POVButton topPov = new POVButton (driver, 0); //POV binding for top POV button, for the "driver" controller 
  private final POVButton topRightPov = new POVButton (driver, 45);//POV button for when top and right are pressed, "driver"
  private final POVButton rightPov = new POVButton (driver, 90);//POV right driver controller
  private final POVButton rightBottomPov = new POVButton (driver, 135);// POV both right and bottom driver controller
  private final POVButton bottomPov = new POVButton (driver, 180);//POV bottom driver
  private final POVButton leftbottomPov = new POVButton (driver, 225);//POV both left and bottom for driver controller
  private final POVButton leftPov = new POVButton (driver, 270);//POV binding for left POV button, for the "driver" controller
  private final POVButton leftTopPov = new POVButton (driver, 315);//POV button for both left and top, driver controller
  private void configureBindings() {
   
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY()* MaxSpeed) // Drive forward with left up/down joystick
            .withVelocityY(-driver.getLeftX()* MaxSpeed) // Drive sideways with left joystick left/right
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with right joystick (left) and clockwise (right)
        ));

      //three vectors for swerve movement within the drivetrain.applyRequest ()


    topPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(.5 * MaxSpeed) // Drive forward with  (forward) @ .5 halfspeed
            .withVelocityY(0* MaxSpeed) // nothing happening with sideways vector
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) // nothing happening on this vector
    );
    
    
    rightPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(.0 * MaxSpeed) // nothing happening on this vector
            .withVelocityY(-.5* MaxSpeed) // Drive right at .5 halfspeed
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) //nothing happening on this vector
    );



    bottomPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * MaxSpeed) // Drive backwards at .5 half speed
            .withVelocityY(.0* MaxSpeed) // nothing happening on this vector
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) // nothing happening on this vector
    );


    leftPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(.0 * MaxSpeed) // nothing happening on this vector
            .withVelocityY(.5* MaxSpeed) // Drive left at .5 halfspeed
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) // nothing happening on this vector
    );

    topRightPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(.5 * MaxSpeed) // Drive forward with  (forward) @ .5 halfspeed
            .withVelocityY(-.5* MaxSpeed) // Drive right at .5 (half speed)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) // nothing happening on this vector
    );

    rightBottomPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * MaxSpeed) // Drive backwards at .5 (halfspeed)
            .withVelocityY(-.5* MaxSpeed) // Drive right at .5 (halfspeed)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) // nothing happening on this vector
    );

    leftbottomPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(-.5 * MaxSpeed) // Drive backwards at -.5 (halfspeed)
            .withVelocityY(.5* MaxSpeed) // Drive left at halfspeed
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) // nothing happening on this vector
    );

    leftTopPov.whileTrue(
  drivetrain.applyRequest(() -> drive.withVelocityX(.5 * MaxSpeed) // Drive forward at .5 (half speed)
            .withVelocityY(.5* MaxSpeed) // Drive left at .5 halfspeed
            .withRotationalRate(-driver.getRightX() * MaxAngularRate)) // nothing happening on this vector

    );

/**
 * left and right trigger buttons for wrist motor
 */
    leftTriggerButton.whileTrue(
      new InstantCommand(wrist::turnClockwise)
    );

    rightTriggerButton.whileTrue(
     new InstantCommand(wrist::turnCounterClockwise) 
    );

    leftTriggerButton.onFalse(
      new InstantCommand(wrist::stopTurning)
    );

    rightTriggerButton.onFalse(
      new InstantCommand(wrist::stopTurning)
    );

/**
 *  buttons for raising and lowering the arm
 */
    rightBumper.whileTrue(
      new InstantCommand(arm::turnClockwise)
    );

    leftBumper.whileTrue(
      new InstantCommand(arm::turnCounterClockwise)
      );

    
    rightBumper.onFalse(
      new InstantCommand(arm::stopTurning)
    );
    
    leftBumper.onFalse(
      new InstantCommand(arm::stopTurning)
    );

/**
 * 
 * Buttons for intake in and out.
 */
    //triangle.whileTrue(
      //new InstantCommand(intake::intaking)
   // );
    /**triangle.onTrue(
      new SequentialCommandGroup(
        new InstantCommand(intake::intaking, intake),
        new WaitCommand(1),
        new InstantCommand(shooter::spinShooter, shooter),
        new WaitCommand(1),
        new InstantCommand(intake::feedToShooter, intake),
        new InstantCommand(shooter::stopShooter, shooter)

      ));
    */




/**
 * 
 * stops intaking or outtaking
 */
    //triangle.onFalse(
     // new InstantCommand(intake::stopIntake)
    //);
    //circle.onFalse(
      //new InstantCommand(intake::stopIntake)
    //);
/**
 * outtaking button method
 * 
 */
    //circle.whileTrue(
      //new InstantCommand(intake::outtaking)
   // );
/**
 * shooter button
 */
  touchpad.whileTrue(
    new InstantCommand(shooter::spinShooter)
  );

/**
 * stop shooter
 */
    touchpad.onFalse(
      new InstantCommand(shooter::stopShooter)
    );

    crossButton.whileTrue(drivetrain.applyRequest(() -> brake)); //sets  to brake mode by making an x
    squareButton.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    //leftBumper.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
   
}
}
  
    
