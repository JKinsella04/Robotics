#pragma config(Sensor, port4,           gyroSensor,     sensorVexIQ_Gyro)
#pragma config(Motor,  motor1,          leftDriveMotor, tmotorVexIQ, PIDControl, driveLeft, encoder)
#pragma config(Motor,  motor3,          leftClawMotor, tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor7,          rightDriveMotor, tmotorVexIQ, PIDControl, reversed, driveRight, encoder)
#pragma config(Sensor, port8,           distanceSensor,     sensorVexIQ_Distance)
#pragma config(Motor,  motor9,          rightClawMotor, tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor10,         middleDriveMotor, tmotorVexIQ, PIDControl, encoder)


static void middleDrive(const int ecount, const int speed)
{
	moveMotorTarget(middleDriveMotor, ecount, speed);
	waitUntilMotorStop(middleDriveMotor);
}

static void moveClaw(const int ecount, const int speed)
{
	moveMotorTarget(leftClawMotor, ecount, speed);
	moveMotorTarget(rightClawMotor, ecount, speed);
	waitUntilMotorStop(leftClawMotor);
}

static void fourWheelDrive(const int ecount, const int speed)
{
	moveMotorTarget(leftDriveMotor, ecount, speed);
	moveMotorTarget(rightDriveMotor, ecount, speed);
	waitUntilMotorStop(leftDriveMotor);
}

static void realign(const float deg, const int speed)
{
	if (getGyroDegreesFloat(gyroSensor) < deg) {
		displaySensorValues(line1, gyroSensor);
		setMotorSpeed(leftDriveMotor, -speed);
		setMotorSpeed(rightDriveMotor, speed);
		waitUntil(getGyroDegreesFloat(gyroSensor) >= deg);
	}
	if (getGyroDegreesFloat(gyroSensor) > deg) {
		displaySensorValues(line1, gyroSensor);
		setMotorSpeed(leftDriveMotor, speed);
		setMotorSpeed(rightDriveMotor, -speed);
		waitUntil(getGyroDegreesFloat(gyroSensor) <= deg);
	}
	stopAllMotors();
	setMotor(middleDriveMotor, -speed);
	wait(1, seconds);
	stopAllMotors();
}

task main()
{
	middleDrive(-35, 100);
	resetGyro(gyroSensor);
	moveClaw(-190, 75);
	fourWheelDrive(950, 50);
	moveClaw(-240, 75);
	fourWheelDrive(-950, 75);
	realign(0.0, 25);
	middleDrive(745, 50);
	fourWheelDrive(-455, 50);
	moveClaw(-230, 75);
	wait(1, seconds);
	moveClaw(150, 100);
	stopAllMotors();
}
