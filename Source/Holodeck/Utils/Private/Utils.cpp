#include "Utils.h"
#include <cmath>

FVector QuatToRotationVector(FQuat quat) {
	FVector axis = quat.GetRotationAxis();
	float angle = quat.GetAngle();

	if (angle < 0 || angle > 6.29) {
		UE_LOG(LogTemp, Warning, TEXT("QuatToRotationVector: Rotation angle error!!"));
	}

	angle = std::fmod(angle + PI, 2 * PI) - PI;

	return axis * angle;
}

FQuat RotationVectorToQuat(FVector rv) {
	float angle = rv.Size();
	FVector axis = rv.GetSafeNormal();
	return FQuat(axis, angle);
}

void printTransform(FTransform t) {

	FQuat quatTemp = t.GetRotation();
	FVector translationTemp = t.GetTranslation();
	FVector scaleTemp = t.GetScale3D();

	UE_LOG(LogTemp, Warning, TEXT("quat  : %f, %f, %f, %f"), quatTemp.W, quatTemp.X, quatTemp.Y, quatTemp.Z);
	UE_LOG(LogTemp, Warning, TEXT("trans : %f, %f, %f"), translationTemp.X, translationTemp.Y, translationTemp.Z);
	UE_LOG(LogTemp, Warning, TEXT("scale : %f, %f, %f"), scaleTemp.X, scaleTemp.Y, scaleTemp.Z);
}

void printVector(FVector v) {
	UE_LOG(LogTemp, Warning, TEXT("vec : %f, %f, %f"), v.X, v.Y, v.Z);
}

void printRotator(FRotator r) {
	UE_LOG(LogTemp, Warning, TEXT("vec : %f, %f, %f"), r.Pitch, r.Yaw, r.Roll);
}

void printQuat(FQuat q) {
	UE_LOG(LogTemp, Warning, TEXT("quat : %f, %f, %f, %f"), q.X, q.Y, q.Z, q.W);
}