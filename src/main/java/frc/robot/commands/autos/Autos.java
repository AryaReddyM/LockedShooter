package frc.robot.commands.autos;

import java.util.Map;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.ActionCommands;
import frc.robot.commands.AutoAlignToPoseCommand;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoCommands.AutoClass;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.State;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.vision.VisionConstants;

public class Autos {

    public static class leftDepotClimb extends AutoClass {
        public leftDepotClimb() {
            this.name = "Left Depot Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Left to Center", "Center to Depot", "Depot to Center", "Center to Climb"};
        }
        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING);
                            }),
                    new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.STOW); 
                            }),
                    new WaitCommand(AutosConstants.shootingPause),
                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.HUB_TRACKING); 
                            }),
                        AutoBuilder.followPath(pathMap.get("Center to Right Fuel (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToRFuel),
                            new InstantCommand(() -> {
                            state.getIntake().requestTransition(Intake.State.INTAKE); 
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.STOW); 
                            }),
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Shoot (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rFuelToShoot),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING);
                            }),
                            new WaitCommand(AutosConstants.shootingPause)
                        )
                    ),
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.HUB_TRACKING); 
                            }),
                            AutoBuilder.followPath(pathMap.get("Shoot to HP (LT)"))
                        )
                    ),
                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            state.getIntake().requestTransition(Intake.State.INTAKE); 
                            }),
                        AutoBuilder.followPath(pathMap.get("HP Pickup (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.hpPickup),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.STOW); 
                            }),
                            new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.SHOOTING);
                            }),
                            new WaitCommand(AutosConstants.shootingPause)
                        )
                    ),
                    new ParallelCommandGroup(
                        new InstantCommand(() -> {
                                state.getShooter().requestTransition(Shooter.State.HUB_TRACKING); 
                            }),
                        AutoBuilder.followPath(pathMap.get("RShoot to Climb (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rShootToClimb)
                        )
                    ),
                    ActionCommands.autoClimb(state)
                    ).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }

    public static class RightFuelHPClimb extends AutoClass { //Done
        public RightFuelHPClimb() {
            this.name = "Right Fuel HP Climb (GAME)";
            this.sequentialPathStrings = new String[] { "Right to Shoot (LT)", 
                                                        "Shoot to Right Fuel (LT)", 
                                                        "Right Fuel to Shoot (LT)", 
                                                        "Shoot to HP (LT)", 
                                                        "HP Pickup (LT)", 
                                                        "RShoot to Climb (LT)"};
        }
        @Override
        public Command getCommand(RobotState state) {
            try {
                Map<String, PathPlannerPath> pathMap = AutoCommands.getMapPath(sequentialPathStrings);
                return new SequentialCommandGroup(
                    new InstantCommand(() -> setRobotPoseToStartingPath(pathMap.get(sequentialPathStrings[0]), state)),
                    new ParallelCommandGroup(
                        AutoBuilder.followPath(pathMap.get("Right to Shoot (LT)")),
                        new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            }),
                        new WaitCommand(AutosConstants.shootingPause)
                    ),
                    new ParallelCommandGroup(
                        //state.getIdle().requestTransition(State.IDLING); // uncomment when idle is ready
                        AutoBuilder.followPath(pathMap.get("Shoot to Right Fuel (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.centerToRFuel),
                            new InstantCommand(() -> {
                                state.getIntake().requestTransition(Intake.State.INTAKE); // uncomment when intake is ready
                            })
                        )
                    ),
                    new ParallelCommandGroup(
                        //state.getIdle().requestTransition(State.IDLING); // uncomment when idle is ready
                        AutoBuilder.followPath(pathMap.get("Right Fuel to Shoot (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rFuelToShoot),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            }),
                            new WaitCommand(AutosConstants.shootingPause)
                        )
                    ),
                    new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            //state.getIdle().requestTransition(State.IDLING); // uncomment when idle is ready
                            AutoBuilder.followPath(pathMap.get("Shoot to HP (LT)"))
                        )
                    ),
                    new ParallelCommandGroup(
                        //state.getIntake().requestTransition(State.INTAKING); // uncomment when intake is ready
                        AutoBuilder.followPath(pathMap.get("HP Pickup (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.hpPickup),
                            new InstantCommand(() -> {
                                //state.getShooter().requestTransition(State.SHOOTING); // uncomment when shooter is ready
                            }),
                            new WaitCommand(AutosConstants.shootingPause)
                        )
                    ),
                    new ParallelCommandGroup(
                        //state.getIdle().requestTransition(State.IDLING); // uncomment when idle is ready
                        AutoBuilder.followPath(pathMap.get("RShoot to Climb (LT)")),
                        new SequentialCommandGroup(
                            new WaitCommand(AutosConstants.rShootToClimb),
                            new InstantCommand(() -> {
                                // state.getClimb().requestTransition(Climb.State.CLIMB); // uncomment when climb is ready
                            })
                        )
                    )).withName(name);
            } catch (Exception e) {
                return new PrintCommand("Failed to generate command: " + e.getMessage()).withName(name + " (FAILED)");
            }
        }
    }
}
