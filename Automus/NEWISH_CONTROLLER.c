#pragma config(Motor,  motor1,          leftDriveMotor, tmotorVexIQ, openLoop, driveLeft, encoder)
#pragma config(Motor,  motor3,          leftArmMotor,  tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor7,          rightDriveMotor, tmotorVexIQ, PIDControl, reversed, driveRight, encoder)
#pragma config(Motor,  motor9,          rightArmMotor, tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor10,         middleDriveMotor, tmotorVexIQ, PIDControl, reversed, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//10769D Wild Cards
task main()
{
	resetMotorEncoder(leftDriveMotor);
	resetMotorEncoder(rightDriveMotor);
	resetMotorEncoder(leftArmMotor);
	resetMotorEncoder(rightArmMotor);
	resetMotorEncoder(middleDriveMotor);

	repeat (forever) {
		displayMotorValues(line1, leftDriveMotor);
		displayMotorValues(line2, rightDriveMotor);
		displayMotorValues(line3, leftArmMotor);
		displayMotorValues(line5, middleDriveMotor);


		tankControl(ChD, ChA, 2);
		armControl(leftArmMotor, BtnLUp, BtnLDown, 50);
		armControl(rightArmMotor, BtnLUp, BtnLDown, 50);
		armControl(middleDriveMotor, BtnEUp, BtnEDown, 75);
		}
}