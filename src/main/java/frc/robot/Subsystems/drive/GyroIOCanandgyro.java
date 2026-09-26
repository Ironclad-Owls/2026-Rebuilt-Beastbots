// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class GyroIOCanandgyro implements GyroIO {
  private final Canandgyro gyro;

  public GyroIOCanandgyro(int canId) {
    // Instantiate the Canandgyro on the provided CAN ID
    gyro = new Canandgyro(canId);

    // Optional: clear any sticky faults from previous runs on boot
    gyro.clearStickyFaults();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    // 1. Update Connection Status
    inputs.connected = gyro.isConnected();

    // 2. Update Yaw/Heading
    // AdvantageKit inputs usually expect a Rotation2d object
    inputs.yawPosition = gyro.getRotation2d();

    // 3. Update Angular Velocity
    // AdvantageKit inputs usually expect Radians Per Second.
    // Canandgyro getRate() returns degrees per second, so we convert it.
    if (inputs.connected) {
      inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getAngularVelocityYaw());
    } else {
      inputs.yawVelocityRadPerSec = 0.0;
    }
    inputs.odometryYawTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
  }
}
