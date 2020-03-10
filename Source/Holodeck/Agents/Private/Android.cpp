// MIT License (c) 2019 BYU PCCL see LICENSE file

#include "Holodeck.h"
#include "Android.h"
#include "Components/PoseableMeshComponent.h"
#include "UObject/UObjectGlobals.h"

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

FVector QuatToRoatationVector(FQuat& quat) {
	FVector axis = quat.GetRotationAxis();
	float angle = quat.GetAngle();

	if (angle < 0 || angle > 6.29) {
		UE_LOG(LogTemp, Warning, TEXT("QuatToRoatationVector: Rotation angle error!!"));
	}

	if (angle > PI) {
		axis = axis * -1.0;
		angle = angle - 2.0 * PI;
	}

	return axis * angle;
}

AAndroid::AAndroid() {
	PrimaryActorTick.bCanEverTick = true;
	bCollisionsAreVisible = false;

	// Set the defualt controller
	AIControllerClass = LoadClass<AController>(NULL, TEXT("/Script/Holodeck.AndroidController"), NULL, LOAD_None, NULL);
	AutoPossessAI = EAutoPossessAI::PlacedInWorld;

	static ConstructorHelpers::FObjectFinder<USkeleton> skeletonObj(TEXT("/Game/HolodeckContent/Agents/AndroidAgent/Character/Mesh/UE4_Mannequin_Skeleton.UE4_Mannequin_Skeleton"));
	if (skeletonObj.Succeeded())
	{
		UE_LOG(LogTemp, Warning, TEXT("AAndroid::AAndroid() load skel succeeded"));
		skeleton = skeletonObj.Object;
	}
	static ConstructorHelpers::FObjectFinder<UAnimSequence> animObj(TEXT("/Game/HolodeckContent/Agents/AndroidAgent/Character/Animations/ThirdPersonWalk.ThirdPersonWalk"));
	
	if (animObj.Succeeded())
	{
		UE_LOG(LogTemp, Warning, TEXT("AAndroid::AAndroid() load anim succeeded"));
		IdleAnim = animObj.Object;
	}
	IdleAnim->SetSkeleton(skeleton);

	cur_time = 0;
}

void AAndroid::InitializeAgent() {
	Super::InitializeAgent();
	SkeletalMesh = Cast<USkeletalMeshComponent>(RootComponent);
	 
	UE_LOG(LogTemp, Warning, TEXT("AAndroid::InitializeAgent()"));

	//body_transform_init.Add(FName("pelvis"), SkeletalMesh->GetBoneTransform(SkeletalMesh->GetBoneIndex(FName("pelvis"))));
	SkeletalMesh->SetPhysicsBlendWeight(0.0);
	SkeletalMesh->SetAllBodiesPhysicsBlendWeight(0.0);
	for (int i = 0; i < NumBodyinstances; i++) {
		auto name = BodyInstanceNames[i];
		//SkeletalMesh->GetBodyInstance(name)->PhysicsBlendWeight = 0.0;
		//SkeletalMesh->GetBodyInstance(name)->SetInstanceSimulatePhysics(false);
		body_transform_init.Add(name, SkeletalMesh->GetBodyInstance(name)->GetUnrealWorldTransform());
		//body_transform_init.Add(ModifiedBoneLists[i], SkeletalMesh->GetBoneTransform(SkeletalMesh->GetBoneIndex(ModifiedBoneLists[i])));
	}



}

void AAndroid::Tick(float DeltaTime) {
	ApplyTorques(DeltaTime);
	Super::Tick(DeltaTime);
	cur_time += DeltaTime;
}

void AAndroid::SetCollisionsVisible(bool Visible) {
	bCollisionsAreVisible = Visible;
}

bool AAndroid::GetCollisionsVisible() {
	return bCollisionsAreVisible;
}

FVector AAndroid::getJointAngularVelocity(FName b_name, FName b_p_name) {
	FVector pv(0, 0, 0);
	FVector v(0, 0, 0);
	if (SkeletalMesh->GetBodyInstance(b_p_name) != NULL) {
		pv = SkeletalMesh->GetBodyInstance(b_p_name)->GetUnrealWorldAngularVelocityInRadians();
	}

	if (SkeletalMesh->GetBodyInstance(b_name) != NULL) {
		v = SkeletalMesh->GetBodyInstance(b_name)->GetUnrealWorldAngularVelocityInRadians();
	}
	return  v - pv;

}


void AAndroid::applyTorqueByName(FName b_name, FName b_p_name, double p_gain, double d_gain) {
	int b_index = SkeletalMesh->GetBoneIndex(b_name);
	int b_p_index = SkeletalMesh->GetBoneIndex(b_p_name);

	FTransform b_transform_init = body_transform_init[b_name];
	FTransform b_p_transform_init = body_transform_init[b_p_name];

	//FTransform b_transform_init, b_p_transform_init;
	//IdleAnim->GetBoneTransform(b_transform_init, b_index, cur_time, true);
	//IdleAnim->GetBoneTransform(b_p_transform_init, b_p_index, cur_time, true);

	FQuat delta_init(b_p_transform_init.GetRotation().Inverse()*b_transform_init.GetRotation());

	FTransform b_transform = SkeletalMesh->GetBoneTransform(b_index);
	FTransform b_p_transform = SkeletalMesh->GetBoneTransform(b_p_index);

	FQuat delta_quat(b_transform.GetRotation()*delta_init.Inverse()*b_p_transform.GetRotation().Inverse());
	FVector delta = QuatToRoatationVector(delta_quat);


	FVector d_delta = getJointAngularVelocity(b_name, b_p_name);

	FVector torque = -1.0 * (delta*(p_gain) + d_delta*d_gain);

	SkeletalMesh->GetBodyInstance(b_name)->SetBodyTransform(b_transform_init, ETeleportType::ResetPhysics, false);
	//SkeletalMesh->AddTorqueInRadians(torque, b_name, true);
}

void AAndroid::ApplyTorques(double DeltaTime) {
	//UE_LOG(LogHolodeck, Warning, TEXT("AAndroid::ApplyTorques, delta time : %f"), DeltaTime);
	int ComInd = 0;
	double p_gain = CommandArray[0] / DeltaTime;
	double d_gain = CommandArray[1] / DeltaTime;

	//UE_LOG(LogTemp, Warning, TEXT("pd gain: %f, %f"), p_gain, d_gain);

	for (int i = 0; i < NumBodyinstances; i++) {
		FTransform bt = body_transform_init[BodyInstanceNames[i]];
		if (i == 0) {
			printTransform(bt);
			//bt.SetTranslation(FVector(0, 1000*cur_time, 0) + bt.GetTranslation());
		}

		SkeletalMesh->GetBodyInstance(BodyInstanceNames[i])->SetBodyTransform(bt, ETeleportType::ResetPhysics, true);
		SkeletalMesh->GetBodyInstance(BodyInstanceNames[i])->SetAngularVelocityInRadians(FVector(0, 0, 0), false);
		SkeletalMesh->GetBodyInstance(BodyInstanceNames[i])->SetLinearVelocity(FVector(0, 0, 0), false);
		SkeletalMesh->GetBodyInstance(BodyInstanceNames[i])->UpdateInstanceSimulatePhysics();
	}
		/*
	for (int i = 0; i < ModifiedNumBones; i++) {
		applyTorqueByName(ModifiedBoneLists[i], ModifiedBoneParentLists[i], p_gain, d_gain);
	}*/
	
	//FTransform tf = SkeletalMesh->GetBoneTransform(SkeletalMesh->GetBoneIndex(FName("spine_01")));
	//Cast<UPoseableMeshComponent>(SkeletalMesh)->SetBoneTransformByName(FName("spine_01"), tf, EBoneSpaces::WorldSpace);

}

const FName AAndroid::Joints[] = {

	// Head, Spine, and Arm joints. Each has [swing1, swing2, twist]
	FName(TEXT("head")),
	FName(TEXT("neck_01")),
	FName(TEXT("spine_02")),
	FName(TEXT("spine_01")),
	FName(TEXT("upperarm_l")),
	FName(TEXT("lowerarm_l")),
	FName(TEXT("hand_l")),
	FName(TEXT("upperarm_r")),
	FName(TEXT("lowerarm_r")),
	FName(TEXT("hand_r")),

	// Leg Joints. Each has [swing1, swing2, twist]
	FName(TEXT("thigh_l")),
	FName(TEXT("calf_l")),
	FName(TEXT("foot_l")),
	FName(TEXT("ball_l")),
	FName(TEXT("thigh_r")),
	FName(TEXT("calf_r")),
	FName(TEXT("foot_r")),
	FName(TEXT("ball_r")),

	// First joint of each finger. Has only [swing1, swing2]
	FName(TEXT("thumb_01_l")),
	FName(TEXT("index_01_l")),
	FName(TEXT("middle_01_l")),
	FName(TEXT("ring_01_l")),
	FName(TEXT("pinky_01_l")),
	FName(TEXT("thumb_01_r")),
	FName(TEXT("index_01_r")),
	FName(TEXT("middle_01_r")),
	FName(TEXT("ring_01_r")),
	FName(TEXT("pinky_01_r")),

	// Second joint of each finger. Has only [swing1]
	FName(TEXT("thumb_02_l")),
	FName(TEXT("index_02_l")),
	FName(TEXT("middle_02_l")),
	FName(TEXT("ring_02_l")),
	FName(TEXT("pinky_02_l")),
	FName(TEXT("thumb_02_r")),
	FName(TEXT("index_02_r")),
	FName(TEXT("middle_02_r")),
	FName(TEXT("ring_02_r")),
	FName(TEXT("pinky_02_r")),

	// Third joint of each finger. Has only [swing1]
	FName(TEXT("thumb_03_l")),
	FName(TEXT("index_03_l")),
	FName(TEXT("middle_03_l")),
	FName(TEXT("ring_03_l")),
	FName(TEXT("pinky_03_l")),
	FName(TEXT("thumb_03_r")),
	FName(TEXT("index_03_r")),
	FName(TEXT("middle_03_r")),
	FName(TEXT("ring_03_r")),
	FName(TEXT("pinky_03_r")),
};

// Don't forget to update AAndroid::NumBones after changing this array!
const FName AAndroid::BoneNames[] = {
	FName(TEXT("pelvis")),
	FName(TEXT("spine_01")),
	FName(TEXT("spine_02")),
	FName(TEXT("spine_03")),
	FName(TEXT("clavicle_l")),
	FName(TEXT("upperarm_l")),
	FName(TEXT("lowerarm_l")),
	FName(TEXT("hand_l")),
	FName(TEXT("index_01_l")),
	FName(TEXT("index_02_l")),
	FName(TEXT("index_03_l")),
	FName(TEXT("middle_01_l")),
	FName(TEXT("middle_02_l")),
	FName(TEXT("middle_03_l")),
	FName(TEXT("pinky_01_l")),
	FName(TEXT("pinky_02_l")),
	FName(TEXT("pinky_03_l")),
	FName(TEXT("ring_01_l")),
	FName(TEXT("ring_02_l")),
	FName(TEXT("ring_03_l")),
	FName(TEXT("thumb_01_l")),
	FName(TEXT("thumb_02_l")),
	FName(TEXT("thumb_03_l")),
	FName(TEXT("lowerarm_twist_01_l")),
	FName(TEXT("upperarm_twist_01_l")),
	FName(TEXT("clavicle_r")),
	FName(TEXT("upperarm_r")),
	FName(TEXT("lowerarm_r")),
	FName(TEXT("hand_r")),
	FName(TEXT("index_01_r")),
	FName(TEXT("index_02_r")),
	FName(TEXT("index_03_r")),
	FName(TEXT("middle_01_r")),
	FName(TEXT("middle_02_r")),
	FName(TEXT("middle_03_r")),
	FName(TEXT("pinky_01_r")),
	FName(TEXT("pinky_02_r")),
	FName(TEXT("pinky_03_r")),
	FName(TEXT("ring_01_r")),
	FName(TEXT("ring_02_r")),
	FName(TEXT("ring_03_r")),
	FName(TEXT("thumb_01_r")),
	FName(TEXT("thumb_02_r")),
	FName(TEXT("thumb_03_r")),
	FName(TEXT("lowerarm_twist_01_r")),
	FName(TEXT("upperarm_twist_01_r")),
	FName(TEXT("neck_01")),
	FName(TEXT("head")),
	FName(TEXT("thigh_l")),
	FName(TEXT("calf_l")),
	FName(TEXT("calf_twist_01_l")),
	FName(TEXT("foot_l")),
	FName(TEXT("ball_l")),
	FName(TEXT("thigh_twist_01_l")),
	FName(TEXT("thigh_r")),
	FName(TEXT("calf_r")),
	FName(TEXT("calf_twist_01_r")),
	FName(TEXT("foot_r")),
	FName(TEXT("ball_r")),
	FName(TEXT("thigh_twist_01_r"))
};

// If you change this number, change the corresponding number in RelativeSkeletalPositionSensor.__init__
const int AAndroid::NumBones = 60;

// Don't forget to update AAndroid::NumBones after changing this array!
const FName AAndroid::BodyInstanceNames[] = {
	FName(TEXT("pelvis")),
	FName(TEXT("spine_01")),
	FName(TEXT("spine_02")),
	FName(TEXT("upperarm_l")),
	FName(TEXT("lowerarm_l")),
	FName(TEXT("hand_l")),
	FName(TEXT("index_01_l")),
	FName(TEXT("index_02_l")),
	FName(TEXT("index_03_l")),
	FName(TEXT("middle_01_l")),
	FName(TEXT("middle_02_l")),
	FName(TEXT("middle_03_l")),
	FName(TEXT("pinky_01_l")),
	FName(TEXT("pinky_02_l")),
	FName(TEXT("pinky_03_l")),
	FName(TEXT("ring_01_l")),
	FName(TEXT("ring_02_l")),
	FName(TEXT("ring_03_l")),
	FName(TEXT("thumb_01_l")),
	FName(TEXT("thumb_02_l")),
	FName(TEXT("thumb_03_l")),
	FName(TEXT("upperarm_r")),
	FName(TEXT("lowerarm_r")),
	FName(TEXT("hand_r")),
	FName(TEXT("index_01_r")),
	FName(TEXT("index_02_r")),
	FName(TEXT("index_03_r")),
	FName(TEXT("middle_01_r")),
	FName(TEXT("middle_02_r")),
	FName(TEXT("middle_03_r")),
	FName(TEXT("pinky_01_r")),
	FName(TEXT("pinky_02_r")),
	FName(TEXT("pinky_03_r")),
	FName(TEXT("ring_01_r")),
	FName(TEXT("ring_02_r")),
	FName(TEXT("ring_03_r")),
	FName(TEXT("thumb_01_r")),
	FName(TEXT("thumb_02_r")),
	FName(TEXT("thumb_03_r")),
	FName(TEXT("neck_01")),
	FName(TEXT("head")),
	FName(TEXT("thigh_l")),
	FName(TEXT("calf_l")),
	FName(TEXT("foot_l")),
	FName(TEXT("ball_l")),
	FName(TEXT("thigh_r")),
	FName(TEXT("calf_r")),
	FName(TEXT("foot_r")),
	FName(TEXT("ball_r")),
};

// If you change this number, change the corresponding number in RelativeSkeletalPositionSensor.__init__
const int AAndroid::NumBodyinstances = 49;

const FName AAndroid::ModifiedBoneLists[] = {
	FName(TEXT("spine_01")),
	FName(TEXT("spine_02")),

	FName(TEXT("upperarm_l")),
	FName(TEXT("lowerarm_l")),
	FName(TEXT("hand_l")),

	FName(TEXT("upperarm_r")),
	FName(TEXT("lowerarm_r")),
	FName(TEXT("hand_r")),

	FName(TEXT("neck_01")),
	FName(TEXT("head")),

	FName(TEXT("thigh_l")),
	FName(TEXT("calf_l")),
	FName(TEXT("foot_l")),
	FName(TEXT("ball_l")),

	FName(TEXT("thigh_r")),
	FName(TEXT("calf_r")),
	FName(TEXT("foot_r")),
	FName(TEXT("ball_r")),
};

const FName AAndroid::ModifiedBoneParentLists[] = {
	FName(TEXT("pelvis")),
	FName(TEXT("spine_01")),

	FName(TEXT("spine_02")),
	FName(TEXT("upperarm_l")),
	FName(TEXT("lowerarm_l")),

	FName(TEXT("spine_02")),
	FName(TEXT("upperarm_r")),
	FName(TEXT("lowerarm_r")),

	FName(TEXT("spine_02")),
	FName(TEXT("neck_01")),

	FName(TEXT("pelvis")),
	FName(TEXT("thigh_l")),
	FName(TEXT("calf_l")),
	FName(TEXT("foot_l")),

	FName(TEXT("pelvis")),
	FName(TEXT("thigh_r")),
	FName(TEXT("calf_r")),
	FName(TEXT("foot_r")),
};

const int AAndroid::ModifiedNumBones = 18;
