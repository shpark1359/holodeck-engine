// MIT License (c) 2019 BYU PCCL see LICENSE file

#include "Holodeck.h"
#include "ReferenceMotion.h"

// Sets default values
AReferenceMotion::AReferenceMotion()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

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

// Called when the game starts or when spawned
void AReferenceMotion::BeginPlay()
{
	Super::BeginPlay();
	root_offset = FVector(-150, 0.0, 28);
	SkeletalMesh = Cast<USkeletalMeshComponent>(GetRootComponent());
	is_animation_loaded = false;
	animation_data.Reset();
	for (int i = 0; i < 1010; i++) {
		TMap<FName, FTransform> animation_frame;
		animation_frame.Reset();
		for (int j = 0; j < NumBodyinstances; j++) {
			FName b_name = BodyInstanceNames[j];
			FTransform bt = GetAnimBoneTransform(b_name, 0.001*i);
			animation_frame.Emplace(b_name, bt);
		}
		animation_data.Emplace(animation_frame);
	}
	is_animation_loaded = true;
}

// Called every frame
void AReferenceMotion::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	for (int j = 0; j < NumBodyinstances; j++) {
		FName b_name = BodyInstanceNames[j];
		FTransform bt = GetAnimBoneTransformWithRoot(b_name, cur_time);
		SkeletalMesh->GetBodyInstance(b_name)->SetBodyTransform(bt, ETeleportType::ResetPhysics);
		SkeletalMesh->GetBodyInstance(b_name)->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		SkeletalMesh->GetBodyInstance(b_name)->SetLinearVelocity(FVector::ZeroVector, false);
	}
}


FTransform AReferenceMotion::GetAnimBoneTransform(FName b_name, float time) {
	time = fmod(time, ((IdleAnim->GetNumberOfFrames() - 1) / IdleAnim->GetFrameRate()));
	if (time < 0) time = time + (IdleAnim->GetNumberOfFrames() - 1) / IdleAnim->GetFrameRate();
	if (is_animation_loaded) {
		return animation_data[(int)(time * 1000)][b_name];
	}
	//UE_LOG(LogTemp, Warning, TEXT("time: %f"), time);
	FTransform res;
	res.SetIdentity();
	FName cur_body_name = b_name;
	while (true) {
		if (cur_body_name == NAME_None) {
			break;
		}
		FTransform tf;
		IdleAnim->GetBoneTransform(tf, SkeletalMesh->GetBoneIndex(cur_body_name), time, true);
		res = res * tf;
		cur_body_name = SkeletalMesh->GetParentBone(cur_body_name);
	}
	return res;
}

FTransform AReferenceMotion::GetAnimBoneTransformWithRootNext(FName b_name) {
	return this->GetAnimBoneTransformWithRoot(b_name, cur_time + time_step);
}
FTransform AReferenceMotion::GetAnimBoneTransformWithRoot(FName b_name) {
	return this->GetAnimBoneTransformWithRoot(b_name, cur_time);
}
FTransform AReferenceMotion::GetAnimBoneTransformWithRoot(FName b_name, float time) {
	FTransform tf = GetAnimBoneTransform(b_name, time);
	tf.SetTranslation(tf.GetTranslation() + FVector(0, 200 * time, 0) + root_offset);
	return tf;
}

// Don't forget to update AAndroid::NumBones after changing this array!
const FName AReferenceMotion::BodyInstanceNames[] = {
	FName(TEXT("pelvis")),
	FName(TEXT("spine_01")),
	FName(TEXT("spine_02")),
	FName(TEXT("upperarm_l")),
	FName(TEXT("lowerarm_l")),
	FName(TEXT("hand_l")),/*
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
	FName(TEXT("thumb_03_l")),*/
	FName(TEXT("upperarm_r")),
	FName(TEXT("lowerarm_r")),
	FName(TEXT("hand_r")),/*
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
	FName(TEXT("thumb_03_r")),*/
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
const int AReferenceMotion::NumBodyinstances = 19;

const int AReferenceMotion::ModifiedNumBones = 18;

const FName AReferenceMotion::ModifiedBoneLists[] = {
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

const FName AReferenceMotion::ModifiedBoneParentLists[] = {
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

