// MIT License (c) 2019 BYU PCCL see LICENSE file

#pragma once

#include "Holodeck.h"

#include "GameFramework/Pawn.h"
#include "HolodeckAgent.h"
#include "Animation/AnimSequence.h"
#include "Components/PoseableMeshComponent.h"

#include "Android.generated.h"

UCLASS()
class HOLODECK_API AAndroid : public AHolodeckAgent
{
	GENERATED_BODY()

public:
	/**
	* Default Constructor
	*/
	AAndroid();

	static constexpr int NUM_JOINTS = 48;
	static constexpr int NUM_3_AXIS_JOINTS = 18;
	static constexpr int NUM_2_AXIS_JOINTS = 10;
	static constexpr int NUM_1_AXIS_JOINTS = 20;
	static constexpr int NUM_2_PLUS_3_AXIS_JOINTS = 28;
	static constexpr int TOTAL_DOF = NUM_3_AXIS_JOINTS * 3 + NUM_2_AXIS_JOINTS * 2 + NUM_1_AXIS_JOINTS; // 94 DOF in total

	const static FName Joints[];
	const static FName BoneNames[];
	const static FName BodyInstanceNames[];
	const static FName ModifiedBoneLists[];
	const static FName ModifiedBoneParentLists[];
	const static int NumBones;
	const static int NumBodyinstances;
	const static int ModifiedNumBones;

	/**
	* Called when the game starts.
	*/
	virtual void InitializeAgent() override;

	/**
	* Tick
	* Called every frame.
	* @param DeltaSeconds the time since the last tick.
	*/
	void Tick(float DeltaSeconds) override;

	//Decal material. This is used to show collisions on the Android. It is to be left blank and is set programmatically
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Materials)
		UMaterial* CollisionDecalMaterial;

	/**
	* SetCollisionsVisible
	* Sets whether collisions with the Android can be seen.
	* @param Visible true to be seen.
	*/
	void SetCollisionsVisible(bool Visible);

	/**
	* GetJoints
	* Gets pointer to constant array of FName joints
	* @return array of FName corresponding to android joint names
	*/
	const FName* GetJoints();

	/**
	* GetCollisionsVisible
	* @return true if collisions are visible.
	*/
	bool GetCollisionsVisible();

	UPROPERTY(BlueprintReadWrite, Category = AndroidMesh)
		USkeletalMeshComponent* SkeletalMesh;

	UAnimSequence* IdleAnim;
	USkeleton* skeleton;

	unsigned int GetRawActionSizeInBytes() const override {
		return TOTAL_DOF * sizeof(float);
	}

	void* GetRawActionBuffer() const override {
		return (void*)CommandArray;
	}
	UPROPERTY(BlueprintReadWrite)
		float cur_time = 0;
		
private:
public:
	bool bCollisionsAreVisible;

	/**
	* ApplyTorques
	* Applies torques for that tick on each joint with a force/direction
	* corresponding to the values in the command array
	*/
	void ResetAgent(float time);
	void ApplyTorques(double DeltaTime);

	void applyTorqueByName(FName b_name, FName b_p_name, double p_gain, double d_gain, FVector action);
	FVector getJointAngle(FName b_name);
	FVector getReferenceJointAngleNext(FName b_name);
	FVector getReferenceJointAngle(FName b_name);
	FVector getReferenceJointAngle(FName b_name, float time);

	FVector getJointAngularVelocity(FName b_name, bool isWorld);
	FVector getReferenceJointAngularVelocity(FName b_name, float time, bool isWorld);
	FVector getReferenceJointAngularVelocity(FName b_name, bool isWorld);

	float CommandArray[TOTAL_DOF];

	FTransform GetAnimBoneTransform(FName b_name, float time);
	FTransform GetAnimBoneTransformWithRoot(FName b_name, float time);
	FTransform GetAnimBoneTransformWithRoot(FName b_name);
	FTransform GetAnimBoneTransformWithRootNext(FName b_name);

	TArray<TMap<FName, FTransform>> animation_data;
	bool is_animation_loaded = false;
	TMap<FName, FTransform> body_transform_init;
	TMap<FName, FVector> torques;
	TMap<FName, FVector> forces;
	TMap<FName, FName> parents;
	FQuat prev_rot;
	float time_step;
	int character_index;
	int step_count;
	UPoseableMeshComponent* mRefComponent;
	FVector root_offset;

};
