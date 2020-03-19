// MIT License (c) 2019 BYU PCCL see LICENSE file

#include "Holodeck.h"
#include "Android.h"
#include "Components/PoseableMeshComponent.h"
#include "UObject/UObjectGlobals.h"
#include "Utils.h"


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
	this->time_step = Cast<AHolodeckWorldSettings>(GetWorld()->GetWorldSettings())->GetConstantTimeDeltaBetweenTicks();
	//body_transform_init.Add(FName("pelvis"), SkeletalMesh->GetBoneTransform(SkeletalMesh->GetBoneIndex(FName("pelvis"))));
	//SkeletalMesh->SetPhysicsBlendWeight(1.0);
	//SkeletalMesh->SetAllBodiesPhysicsBlendWeight(1.0);
	for (int i = 0; i < NumBodyinstances; i++) {
		auto name = BodyInstanceNames[i];
		body_transform_init.Add(name, SkeletalMesh->GetBodyInstance(name)->GetUnrealWorldTransform());

		if (this->GetName() == "AndroidBlueprint_C_1") {
			SkeletalMesh->GetBodyInstance(name)->SetInstanceSimulatePhysics(false);
		}
		if (this->GetName() == "AndroidBlueprint_C_0") {
			SkeletalMesh->GetBodyInstance(name)->SetInstanceSimulatePhysics(true);
			FTransform bt = GetAnimBoneTransform(name, 0);
			bt.SetTranslation(bt.GetTranslation() + FVector(-100, 0, 30));
			SkeletalMesh->GetBodyInstance(name)->SetBodyTransform(bt, ETeleportType::ResetPhysics);
			SkeletalMesh->GetBodyInstance(name)->SetAngularVelocityInRadians(FVector(0, 0, 0), false);
			SkeletalMesh->GetBodyInstance(name)->SetLinearVelocity(FVector(0, 0, 0), false);
		}
	}

	parents.Reset();
	for (int i = 0; i < ModifiedNumBones; i++) {
		parents.Add(ModifiedBoneLists[i], ModifiedBoneParentLists[i]);
	}



}

void AAndroid::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
	ApplyTorques(DeltaTime);
	cur_time += DeltaTime;
}

void AAndroid::SetCollisionsVisible(bool Visible) {
	bCollisionsAreVisible = Visible;
}

bool AAndroid::GetCollisionsVisible() {
	return bCollisionsAreVisible;
}

// get joint angle in joint coordinate
FVector AAndroid::getJointAngle(FName b_name) {
	FName b_p_name = this->parents[b_name];
	FTransform b_p_transform = SkeletalMesh->GetBodyInstance(b_p_name)->GetUnrealWorldTransform();
	FTransform b_transform = SkeletalMesh->GetBodyInstance(b_name)->GetUnrealWorldTransform();
	return QuatToRotationVector(b_p_transform.GetRotation().Inverse() * b_transform.GetRotation());

}

FVector AAndroid::getReferenceJointAngleNext(FName b_name) {
	return getReferenceJointAngle(b_name, cur_time + time_step);
}
FVector AAndroid::getReferenceJointAngle(FName b_name) {
	return getReferenceJointAngle(b_name, cur_time);
}
FVector AAndroid::getReferenceJointAngle(FName b_name, float time) {
	FName b_p_name = this->parents[b_name];
	FTransform b_p_transform = this->GetAnimBoneTransform(b_p_name, time);
	FTransform b_transform = this->GetAnimBoneTransform(b_name, time);
	return QuatToRotationVector(b_p_transform.GetRotation().Inverse() * b_transform.GetRotation());

}
FVector AAndroid::getJointAngularVelocity(FName b_name) {
	FName b_p_name = this->parents[b_name];
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

FVector AAndroid::getReferenceJointAngularVelocity(FName b_name) {
	return getReferenceJointAngularVelocity(b_name, cur_time);
}

FVector AAndroid::getReferenceJointAngularVelocity(FName b_name, float time) {
	FName b_p_name = this->parents[b_name];
	FTransform b_transform_cur = GetAnimBoneTransform(b_name, time);
	FTransform b_p_transform_cur = GetAnimBoneTransform(b_p_name, time);

	FTransform b_transform_prev = GetAnimBoneTransform(b_name, time - time_step);
	FTransform b_p_transform_prev = GetAnimBoneTransform(b_p_name, time - time_step);

	return (QuatToRotationVector(b_transform_cur.GetRotation()*b_transform_prev.GetRotation().Inverse())
		- QuatToRotationVector(b_p_transform_cur.GetRotation()*b_p_transform_prev.GetRotation().Inverse())) / time_step;


}
FTransform AAndroid::GetAnimBoneTransform(FName b_name, float time) {
	time = fmod(time, (IdleAnim->GetRawNumberOfFrames() / IdleAnim->GetFrameRate()));
	//UE_LOG(LogTemp, Warning, TEXT("time: %f"), time);
	FTransform res;
	res.SetIdentity();
	FName cur_body_name = b_name;
	while(true){
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

FTransform AAndroid::GetAnimBoneTransformWithRootNext(FName b_name) {
	return this->GetAnimBoneTransformWithRoot(b_name, cur_time + time_step);
}
FTransform AAndroid::GetAnimBoneTransformWithRoot(FName b_name) {
	return this->GetAnimBoneTransformWithRoot(b_name, cur_time);
}
FTransform AAndroid::GetAnimBoneTransformWithRoot(FName b_name, float time) {
	FTransform tf = GetAnimBoneTransform(b_name, time);
	tf.SetTranslation(tf.GetTranslation() + FVector(0, 200 * time, 0));
	return tf;
}

void AAndroid::applyTorqueByName(FName b_name, FName b_p_name, double p_gain, double d_gain) {
	int b_index = SkeletalMesh->GetBoneIndex(b_name);
	int b_p_index = SkeletalMesh->GetBoneIndex(b_p_name);
	
	//FTransform b_transform_init = body_transform_init[b_name];
	//FTransform b_p_transform_init = body_transform_init[b_p_name];
	FTransform b_transform_cur = GetAnimBoneTransform(b_name, cur_time);
	FTransform b_p_transform_cur = GetAnimBoneTransform(b_p_name, cur_time);

	FQuat delta_cur(b_p_transform_cur.GetRotation().Inverse()*b_transform_cur.GetRotation());

	FTransform b_transform = SkeletalMesh->GetBoneTransform(b_index);
	FTransform b_p_transform = SkeletalMesh->GetBoneTransform(b_p_index);

	FQuat delta_quat(b_p_transform.GetRotation().Inverse()*b_transform.GetRotation()*delta_cur.Inverse());
	FVector delta = b_p_transform.GetRotation().RotateVector(QuatToRotationVector(delta_quat));


	FVector cur_vel = getJointAngularVelocity(b_name);

	FTransform b_transform_prev = GetAnimBoneTransform(b_name, cur_time - time_step);
	FTransform b_p_transform_prev = GetAnimBoneTransform(b_p_name, cur_time - time_step);

	FVector desired_vel2 = (QuatToRotationVector(b_transform_cur.GetRotation()*b_transform_prev.GetRotation().Inverse())
		- QuatToRotationVector(b_p_transform_cur.GetRotation()*b_p_transform_prev.GetRotation().Inverse())) / time_step;


	FVector d_delta = cur_vel - desired_vel2;

	FVector torque = -1.0 * (delta*p_gain + d_delta*d_gain);
	/*
	if (b_name == FName("spine_01")) {
		UE_LOG(LogTemp, Warning, TEXT("time: %f"), cur_time);
		printVector(torque);
		printVector(delta);
		printVector(d_delta);
		UE_LOG(LogTemp, Warning, TEXT("pd gain: %f, %f"), p_gain, d_gain);
	}
	
	if (b_name == FName("upperarm_r")) {
		UKismetSystemLibrary::DrawDebugArrow(this->GetWorld(),
			SkeletalMesh->GetBodyInstance(b_name)->GetUnrealWorldTransform().GetTranslation(),
			SkeletalMesh->GetBodyInstance(b_name)->GetUnrealWorldTransform().GetTranslation()+FVector(0,0,50),
			1,
			FLinearColor(1, 0, 0),
			0.02,
			1
		);
		UKismetSystemLibrary::DrawDebugArrow(this->GetWorld(),
			SkeletalMesh->GetBodyInstance(b_name)->GetCOMPosition(),
			SkeletalMesh->GetBodyInstance(b_name)->GetCOMPosition() + FVector(0, 0, 50),
			1,
			FLinearColor(0, 1, 0),
			0.02,
			1
		);
	}
	*/

	torques[b_p_name] -= torque;
	torques[b_name] += torque;
	//SkeletalMesh->AddTorqueInRadians(-torque, b_p_name, true);
	//SkeletalMesh->AddTorqueInRadians(torque, b_name, true);
}

void AAndroid::ApplyTorques(double DeltaTime) {
	UE_LOG(LogHolodeck, Warning, TEXT("AAndroid::ApplyTorques, delta time : %f"), DeltaTime);
	int ComInd = 0;
	double p_gain = CommandArray[0];
	double d_gain = CommandArray[1];

	//UE_LOG(LogTemp, Warning, TEXT("pd gain: %f, %f"), p_gain, d_gain);

	
	if (cur_time == 0 && this->GetName() == "AndroidBlueprint_C_1") {
		SkeletalMesh->PlayAnimation(IdleAnim, true);
	}

	if (this->GetName() == "AndroidBlueprint_C_1") {
		FTransform at = GetActorTransform();
		at.SetTranslation(at.GetTranslation() + FVector(0, 200*DeltaTime, 0));
		SetActorTransform(at);
	}
	
	/*
	if (this->GetName() == "AndroidBlueprint_C_0") {
		for (int i = 0; i < NumBodyinstances; i++) {
			if (BodyInstanceNames[i] == FName("pelvis")) {
				FTransform bt = GetAnimBoneTransform(BodyInstanceNames[i], 0.0);
				bt.SetTranslation(bt.GetTranslation() + FVector(-100, 0, 50));

				SkeletalMesh->GetBodyInstance(BodyInstanceNames[i])->SetBodyTransform(bt, ETeleportType::ResetPhysics, true);
				SkeletalMesh->GetBodyInstance(BodyInstanceNames[i])->SetAngularVelocityInRadians(FVector(0, 0, 0), false);
				SkeletalMesh->GetBodyInstance(BodyInstanceNames[i])->SetLinearVelocity(FVector(0, 0, 0), false);
			}
		}
	}
	*/
	
	torques.Reset();
	forces.Reset();
	torques.Add(FName("pelvis"), FVector(0.0, 0.0, 0.0));
	forces.Add(FName("pelvis"), FVector(0.0, 0.0, 0.0));
	for (int i = 0; i < ModifiedNumBones; i++) {
		torques.Add(ModifiedBoneLists[i], FVector(0.0, 0.0, 0.0));
		forces.Add(ModifiedBoneLists[i], FVector(0.0, 0.0, 0.0));
	}
	for (int i = 0; i < ModifiedNumBones; i++) {
		applyTorqueByName(ModifiedBoneLists[i], ModifiedBoneParentLists[i], p_gain, d_gain);
	}

	//UE_LOG(LogTemp, Warning, TEXT("==========================="));
	//printVector(torques[FName("pelvis")]);
	for (auto& elem : torques) {

		// torque to force and twist
		FVector com_pos = SkeletalMesh->GetBodyInstance(elem.Key)->GetCOMPosition();
		FVector joint_pos = SkeletalMesh->GetBodyInstance(elem.Key)->GetUnrealWorldTransform().GetTranslation();
		FVector r = com_pos - joint_pos;
		FVector twist_torque = FVector::DotProduct(elem.Value, r.GetSafeNormal()) * r.GetSafeNormal();
		FVector swing_torque = elem.Value - twist_torque;
		FVector swing_force = swing_torque.Size() / r.Size() * FVector::CrossProduct(swing_torque.GetSafeNormal(), r.GetSafeNormal());
		//printVector(elem.Value);
		// twist torque to forces
		{
			FVector dr = FVector::CrossProduct(r, FVector(1, 0, 0));
			if (dr.Size() < 0.001) {
				dr = FVector::CrossProduct(r, FVector(0, 1, 0));
			}
			dr = dr.GetSafeNormal();

			FVector twist_force = 0.5 * twist_torque.Size() / dr.Size() * FVector::CrossProduct(twist_torque.GetSafeNormal(), dr.GetSafeNormal());
			SkeletalMesh->GetBodyInstance(elem.Key)->AddForceAtPosition(twist_force, com_pos + dr);
			SkeletalMesh->GetBodyInstance(elem.Key)->AddForceAtPosition(-twist_force, com_pos - dr);
		}
		//SkeletalMesh->GetBodyInstance(elem.Key)->AddTorqueInRadians(twist_torque);
		SkeletalMesh->GetBodyInstance(elem.Key)->AddForceAtPosition(swing_force, com_pos);
		SkeletalMesh->GetBodyInstance(elem.Key)->AddForceAtPosition(-swing_force, joint_pos);
	}
	

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

const int AAndroid::ModifiedNumBones = 18;

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

