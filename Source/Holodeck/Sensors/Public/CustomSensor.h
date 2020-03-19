#pragma once

#include "Holodeck.h"
#include "Android.h"
#include "HandAgent.h"

#include "HolodeckPawnController.h"
#include "HolodeckSensor.h"
#include "PhysicsEngine/ConstraintInstance.h"

#include "CustomSensor.generated.h"

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class HOLODECK_API UCustomSensor : public UHolodeckSensor {
	GENERATED_BODY()

public:
	/**
	* Default Constructor.
	*/
	UCustomSensor();

	/**
	* InitializeSensor
	* Sets up the class
	*/
	virtual void InitializeSensor() override;

protected:
	// See HolodeckSensor for information on these classes.
	virtual int GetNumItems() override;
	virtual int GetItemSize() override { return sizeof(float); };
	virtual void TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	USkeletalMeshComponent* SkeletalMeshComponent;
	AAndroid* Parent;

	int NumJoints, NumEEs;
	int StateSize, RewardSize, TotalSize;

	void GetState();
	void GetReward();

	TArray<FName> EEList;
};
