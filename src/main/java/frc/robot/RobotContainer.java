package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.AutonomousOptions;
import frc.lib.Constants;
import frc.robot.classes.PhotonCameraHandler;
import frc.robot.classes.handlers.BeamBreakHandler;
import frc.robot.classes.handlers.ToggleHandler;

import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.subsystems.*;
public class RobotContainer {
    /* Controllers */
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);




    /* Instatiantion of Triggers */

    private final int LeftYAxis = XboxController.Axis.kLeftY.value;
    private final int LeftXAxis = XboxController.Axis.kLeftX.value;
    private final int RightYAxis = XboxController.Axis.kRightY.value;
    private final int RightXAxis = XboxController.Axis.kRightX.value;

    private final int RightTriggerAxis = XboxController.Axis.kRightTrigger.value;
    private final int LeftTriggerAxis = XboxController.Axis.kLeftTrigger.value;

    private final Trigger pilotRightTrigger = pilot.rightTrigger();
    private final Trigger pilotLeftTrigger = pilot.leftTrigger();
    private final Trigger copilotRightTrigger = copilot.rightTrigger();
    private final Trigger copilotLeftTrigger = copilot.leftTrigger();

    private final Trigger pilotRightBumper = pilot.rightBumper();
    private final Trigger pilotLeftBumper = pilot.leftBumper();
    private final Trigger copilotRightBumper = copilot.rightBumper();
    private final Trigger copilotLeftBumper = copilot.leftBumper();

    private final Trigger pilotyButton = pilot.y();
    private final Trigger pilotaButton = pilot.a();
    private final Trigger pilotbButton = pilot.b();
    private final Trigger pilotxButton = pilot.x();
    private final Trigger copilotaButton = copilot.a();
    private final Trigger copilotbButton = copilot.b();
    private final Trigger copilotxButton = copilot.x();

    private final Trigger copilotPOVup = copilot.povUp();
    private final Trigger copilotPOVleft = copilot.povLeft();
    private final Trigger copilotPOVright = copilot.povRight();
    private final Trigger copilotPOVdown = copilot.povDown();


    private final BeamBreakHandler c_BeamBreak;
    private final PhotonCameraHandler c_PhotonCamera1;

    private final ToggleHandler beamBreakDisable;

    /* Subsystems */
    private final Swerve s_Swerve;


    public RobotContainer() {

        beamBreakDisable = new ToggleHandler("BEAMBREAK_DISABLE");


        c_BeamBreak = new BeamBreakHandler(Constants.beambreakconfig, beamBreakDisable);
        c_PhotonCamera1 = new PhotonCameraHandler(Constants.camera1Config);

        s_Swerve = new Swerve(c_PhotonCamera1);

        AutoBuilder.configure(
                s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::getRobotChassisSpeeds,
                (speeds, feedforwards) -> s_Swerve.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(s_Swerve.PATHPLANNER_TRANSLATION_P.get(), s_Swerve.PATHPLANNER_TRANSLATION_I.get(), s_Swerve.PATHPLANNER_TRANSLATION_D.get()), // Translation PID constants
                        new PIDConstants(s_Swerve.PATHPLANNER_ROTATION_P.get(), s_Swerve.PATHPLANNER_ROTATION_I.get(), s_Swerve.PATHPLANNER_ROTATION_D.get()) // Rotation PID constants
                ),
                Constants.robotConfig,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                s_Swerve);


        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -(pilot.getRawAxis(LeftYAxis) - (pilot.getRawAxis(LeftYAxis))),
                () -> -(pilot.getRawAxis(LeftXAxis) - (pilot.getRawAxis(LeftXAxis))),
                () -> (pilot.getRawAxis(RightXAxis) - (pilot.getRawAxis(RightXAxis))),
                    pilotLeftBumper::getAsBoolean
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

    public Command getAutonomousCommand(AutonomousOptions plan) {
        return new InstantCommand();
    }

}
