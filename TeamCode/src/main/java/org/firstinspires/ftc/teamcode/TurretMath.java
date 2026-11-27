package org.firstinspires.ftc.teamcode;

public class TurretMath {

// Calculate the turret's desired angle
public static double calculateTurretAngle(
double robotX,
double robotY,
double robotHeading,
double targetX,
double targetY
) {

double dx = targetX - robotX;
double dy = targetY - robotY;

// Calculate field-relative bearing
double bearing = Math.toDegrees(
Math.atan2(dx, dy)
);

// Convert to robot-relative angle
double turretAngle = bearing - robotHeading;

// Keep angle between -180 and +180
while (turretAngle > 180) {
turretAngle -= 360;
}

while (turretAngle <= -180) {
turretAngle += 360;
}

return turretAngle;
}


// Calculate distance from robot to target
public static double calculateDistance(
double robotX,
double robotY,
double targetX,
double targetY
) {

double dx = targetX - robotX;
double dy = targetY - robotY;

return Math.sqrt(dx * dx + dy * dy);
}
}