#include "autons/auton_functions.h"
#include "utility/pid_control.h"
#include "robot-config.h"
#include "main.h"
#include "iostream"

double newTurnVelocity;

namespace auton {
    double tileDistanceInch = 23.5625;
    double baseWheelDiameter = 3.5;
    double wheelGearTeeth = 48.0;
    double motorGearTeeth = 36.0;

    double heading_convert(double heading) {
        return (heading > 180) ? heading - 360 : heading;
    }

    double sped = 0.04; // Decreased initial Kp

    void driveAndTurn(double tiles, double angle, double linearMaxVelocity, double turnMaxVelocity, double timeoutMs) {
        double distanceDegree = tiles * (tileDistanceInch / 1.0) * (1.0 / (M_PI * baseWheelDiameter)) * (wheelGearTeeth / 1.0) * (1.0 / motorGearTeeth) * (360.0 / 1.0);
        double initLeftMotorDegree = leftmo.position(degrees);
        double initRightMotorDegree = rightmo.position(degrees);

        PIDControl drivePID(sped, 0.01, 0.02, 2); // Added Ki and Kd
        PIDControl rotateToPID(0.45, 0, 0.02, 2); // Added Kd for turn PID

        double initialHeading = bob.rotation(degrees);

        timer timeout;
        while(timeout.time(msec) <= timeoutMs && (!drivePID.reachedGoal() || !rotateToPID.reachedGoal())) {
            // Get linear velocity
            double currentLeftMotorDegree = leftmo.position(degrees);
            double currentRightMotorDegree = rightmo.position(degrees);
            double leftTraveledDegree = currentLeftMotorDegree - initLeftMotorDegree;
            double rightTraveledDegree = currentRightMotorDegree - initRightMotorDegree;
            double averageTraveledDegree = (leftTraveledDegree + rightTraveledDegree) / 2;
            double error = distanceDegree - averageTraveledDegree;
            drivePID.computeFromError(error);
            double newLinearVelocity = drivePID.getValue();
            newLinearVelocity = fmax(-linearMaxVelocity, fmin(linearMaxVelocity, newLinearVelocity));

            // Get turning velocity
            double currentHeading = bob.rotation(degrees);
            double headingError = angle - heading_convert(currentHeading - initialHeading);
            rotateToPID.computeFromError(headingError);
            newTurnVelocity = rotateToPID.getValue();
            newTurnVelocity = fmax(-turnMaxVelocity, fmin(turnMaxVelocity, newTurnVelocity));

            // Get final velocity
            double finalLeftVelocity = newLinearVelocity + newTurnVelocity;
            double finalRightVelocity = newLinearVelocity - newTurnVelocity;
            driveVelocity(finalLeftVelocity, finalRightVelocity);

            task::sleep(20);
        }
        leftmo.stop();
        rightmo.stop();
    }

    void turnToAngle(double angle, double MaxVelocity, double timeoutMs) {
        PIDControl rotateToPID(0.65, 0, 0.02, 2); // Adjusted Kd
        timer timeout;
        while(timeout.time(msec) <= timeoutMs && !rotateToPID.reachedGoal()) {
            double error = angle - heading_convert(bob.rotation(degrees));
            rotateToPID.computeFromError(error);
            double newTurnVelocity = rotateToPID.getValue(); 
            driveVelocity(newTurnVelocity, -newTurnVelocity);
            printf("error%3f\n", error); 
            task::sleep(20); 
        }
        leftmo.stop();
        rightmo.stop();
    }

    void driveVelocity(double leftPct, double rightPct) {
        double scale = 100.0 / fmax(100.0, fmax(fabs(leftPct), fabs(rightPct)));
        leftPct *= scale;
        rightPct *= scale;
        leftmo.spin(fwd, leftPct, pct);
        rightmo.spin(fwd, rightPct, pct);
    }

    void setHeading(int degree) {
        bob.setHeading(degree, degrees);
    }

    void suk(int speed) {
        inta.spin(reverse, speed, pct);
    }

    void unsuk(int speed) {
        inta.spin(fwd, speed, pct);
    }
}
