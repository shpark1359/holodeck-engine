#pragma once
#include "Holodeck.h"

FVector QuatToRotationVector(FQuat quat);
FQuat RotationVectorToQuat(FVector rv);

void printTransform(FTransform t);
void printVector(FVector v);
void printRotator(FRotator r);
void printQuat(FQuat q);