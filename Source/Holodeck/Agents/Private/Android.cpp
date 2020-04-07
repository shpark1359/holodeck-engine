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
	this->time_step = 0.002 * 20;// Cast<AHolodeckWorldSettings>(GetWorld()->GetWorldSettings())->GetConstantTimeDeltaBetweenTicks();
	//SkeletalMesh->SetPhysicsBlendWeight(1.0);
	//SkeletalMesh->SetAllBodiesPhysicsBlendWeight(1.0);
	UE_LOG(LogTemp, Warning, TEXT("AAndroid::InitializeAgent() name : %s"), *this->GetName());
	TArray<FString> splited_name;
	int splited_num = this->GetName().ParseIntoArray(splited_name, TEXT("_"));
	this->character_index = FCString::Atoi(*splited_name[2]);

	for (int i = 0; i < NumBodyinstances; i++) {
		auto name = BodyInstanceNames[i];
		root_offset = FVector(250*this->character_index, 0, 28);
		SkeletalMesh->GetBodyInstance(name)->SetInstanceSimulatePhysics(true);
	}

	ResetAgent(0.0);
	cur_time = 0.0;

	parents.Reset();
	for (int i = 0; i < ModifiedNumBones; i++) {
		parents.Add(ModifiedBoneLists[i], ModifiedBoneParentLists[i]);
	}

	for (int j = 0; j < NumBodyinstances; j++) {
		FName b_name = BodyInstanceNames[j];
		UE_LOG(LogTemp, Warning, TEXT("%s : %f"), *b_name.ToString(), SkeletalMesh->GetBodyInstance(b_name)->GetBodyMass());
	}
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


	// DEBUG 
	for (int j = 0; j < NumBodyinstances; j++) {
		FName b_name = BodyInstanceNames[j];
		float mav = SkeletalMesh->GetBodyInstance(b_name)->PhysicsBlendWeight;
		UE_LOG(LogTemp, Warning, TEXT("%s : %f"), *b_name.ToString(), mav);
		SkeletalMesh->GetBodyInstance(b_name)->SetMaxAngularVelocityInRadians(10, false);
	}


	// add component for reference
	//mRefComponent = Cast<UPoseableMeshComponent>(RootComponent->GetChildComponent(0));
	//UE_LOG(LogTemp, Warning, TEXT("reference Component name : %s"), *(mRefComponent->GetD()));
}

void AAndroid::Tick(float DeltaTime) {
	Super::Tick(DeltaTime);
	if (!is_animation_loaded) return;
	float reset_required = CommandArray[0];
	if (reset_required > 0.5) {
		float reset_time = int(1000 * CommandArray[1]) / 1000.;
		ResetAgent(reset_time);
		cur_time = reset_time;
		step_count = 0;
	}
	else {

		ApplyTorques(DeltaTime);
		step_count++;
		if (step_count == 20) {
			step_count = 0;
			cur_time += this->time_step;
		}
	}

	// set reference
	/*
	for (int j = 0; j < NumBodyinstances; j++) {
		FName b_name = BodyInstanceNames[j];
		FTransform bt = GetAnimBoneTransformWithRoot(b_name, cur_time);
		SkeletalMesh->GetBodyInstance(b_name)->SetBodyTransform(bt, ETeleportType::ResetPhysics);
		SkeletalMesh->GetBodyInstance(b_name)->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		SkeletalMesh->GetBodyInstance(b_name)->SetLinearVelocity(FVector::ZeroVector, false);
		bt.SetTranslation(bt.GetTranslation() + FVector(-100, 0.0, 0.0));
		if (b_name == FName("pelvis")) {
			UE_LOG(LogTemp, Warning, TEXT("pelvis translation : %f, %f, %f "), bt.GetTranslation().X, bt.GetTranslation().Y, bt.GetTranslation().Z);
		}
		mRefComponent->SetBoneTransformByName(b_name, bt, EBoneSpaces::WorldSpace);

		if (b_name == FName("ball_l")) {
			FVector trans = bt.GetTranslation();
			FVector skel_get = SkeletalMesh->GetBodyInstance(b_name)->GetUnrealWorldTransform().GetTranslation();
			FVector ref_get = mRefComponent->GetBoneTransform(mRefComponent->GetBoneIndex(b_name)).GetTranslation();
			UE_LOG(LogTemp, Warning, TEXT("skel set : (%f, %f, %f)"), trans[0] + 100, trans[1], trans[2]);
			UE_LOG(LogTemp, Warning, TEXT("skel get : (%f, %f, %f)"), skel_get[0], skel_get[1], skel_get[2]);

			UE_LOG(LogTemp, Warning, TEXT("ref  set : (%f, %f, %f)"), trans[0], trans[1], trans[2]);
			UE_LOG(LogTemp, Warning, TEXT("ref  get : (%f, %f, %f)"), ref_get[0], ref_get[1], ref_get[2]);
		}

	}
	// debug angular velocities
	UE_LOG(LogTemp, Warning, TEXT("======================================"));
	for (int i = 0; i < ModifiedNumBones; i++) {
		auto name = ModifiedBoneLists[i];
		//FVector av = SkeletalMesh->GetBodyInstance(name)->GetUnrealWorldAngularVelocityInRadians();
		FVector av = getJointAngularVelocity(name);
		FVector rav = getReferenceJointAngularVelocity(name);
		float diff = (av - rav).Size();
		UE_LOG(LogTemp, Warning, TEXT("name : %s : ref (%f, %f, %f), sim (%f, %f, %f) - diff : %f"),
			*name.ToString(),
			rav[0], rav[1], rav[2],
			av[0], av[1], av[2],
			diff
		);
	}
	*/
	
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
	float vel_time_step = 0.01;
	FTransform b_transform_cur = GetAnimBoneTransform(b_name, time);
	FTransform b_p_transform_cur = GetAnimBoneTransform(b_p_name, time);

	FTransform b_transform_prev = GetAnimBoneTransform(b_name, time - vel_time_step);
	FTransform b_p_transform_prev = GetAnimBoneTransform(b_p_name, time - vel_time_step);

	return (QuatToRotationVector(b_transform_cur.GetRotation()*b_transform_prev.GetRotation().Inverse())
		- QuatToRotationVector(b_p_transform_cur.GetRotation()*b_p_transform_prev.GetRotation().Inverse())) / vel_time_step;


}
FTransform AAndroid::GetAnimBoneTransform(FName b_name, float time) {
	time = fmod(time, ((IdleAnim->GetNumberOfFrames()-1) / IdleAnim->GetFrameRate()));
	if (time < 0) time = time + (IdleAnim->GetNumberOfFrames()-1) / IdleAnim->GetFrameRate();
	if (is_animation_loaded) {
		return animation_data[(int)(time * 1000)][b_name];
	}
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
	tf.SetTranslation(tf.GetTranslation() + FVector(0, 200 * time, 0) + root_offset);
	return tf;
}

void AAndroid::applyTorqueByName(FName b_name, FName b_p_name, double p_gain, double d_gain, FVector action) {
	int b_index = SkeletalMesh->GetBoneIndex(b_name);
	int b_p_index = SkeletalMesh->GetBoneIndex(b_p_name);
	
	float vel_time_step = 0.01;

	//FTransform b_transform_init = body_transform_init[b_name];
	//FTransform b_p_transform_init = body_transform_init[b_p_name];
	FTransform b_transform_cur = GetAnimBoneTransform(b_name, cur_time + time_step);
	FTransform b_p_transform_cur = GetAnimBoneTransform(b_p_name, cur_time + time_step);

	FQuat delta_cur(b_p_transform_cur.GetRotation().Inverse()*b_transform_cur.GetRotation());

	FTransform b_transform = SkeletalMesh->GetBoneTransform(b_index);
	FTransform b_p_transform = SkeletalMesh->GetBoneTransform(b_p_index);


	FQuat delta_target(b_p_transform.GetRotation().Inverse()*b_transform.GetRotation());
	delta_cur = RotationVectorToQuat(QuatToRotationVector(delta_cur) + action);
	FQuat delta_quat(delta_target*delta_cur.Inverse());
	FVector delta = b_p_transform.GetRotation().RotateVector(QuatToRotationVector(delta_quat));


	FVector cur_vel = getJointAngularVelocity(b_name);

	FTransform b_transform_prev = GetAnimBoneTransform(b_name, cur_time + time_step - vel_time_step);
	FTransform b_p_transform_prev = GetAnimBoneTransform(b_p_name, cur_time + time_step - vel_time_step);

	FVector desired_vel = (QuatToRotationVector(b_transform_cur.GetRotation()*b_transform_prev.GetRotation().Inverse())
		- QuatToRotationVector(b_p_transform_cur.GetRotation()*b_p_transform_prev.GetRotation().Inverse())) / vel_time_step;


	FVector d_delta = b_transform.GetRotation().RotateVector(
		b_transform.GetRotation().Inverse().RotateVector(cur_vel) 
		- b_transform_cur.GetRotation().Inverse().RotateVector(desired_vel)
	);

	if (b_name == FName("ball_l") || b_name == FName("ball_r") || b_name == FName("hand_l") || b_name == FName("hand_r")) {
		p_gain *= 0.5;
		d_gain *= 0.5;
	}
	else if ( b_name == FName("head")) {
		p_gain *= 1;
		d_gain *= 1;
	}
	else if (b_name == FName("lowerarm_l") || b_name == FName("lowerarm_r") || b_name == FName("foot_l") || b_name == FName("foot_r") || b_name == FName("neck_01")) {
		p_gain *= 2;
		d_gain *= 2;
	}
	else if (b_name == FName("calf_l") || b_name == FName("calf_r") || b_name == FName("upperarm_l") || b_name == FName("upperarm_r")) {
		p_gain *= 3;
		d_gain *= 3;
	}
	else if (b_name == FName("spine_01") || b_name == FName("spine_02") || b_name == FName("thigh_l") || b_name == FName("thigh_r")) {
		p_gain *= 4;
		d_gain *= 4;
	}
	else {
		UE_LOG(LogTemp, Error, TEXT("Unspeicified bone name!"));
	}

	FVector torque = -1.0 * (delta*p_gain + d_delta*d_gain);

	torques[b_p_name] -= torque;
	torques[b_name] += torque;


	
}


void AAndroid::ResetAgent(float reset_time) {
	for (int i = 0; i < NumBodyinstances; i++) {
		float vel_time_step = 0.01;
		auto name = BodyInstanceNames[i];
		FTransform bt = GetAnimBoneTransformWithRoot(name, reset_time);
		FTransform bt_prev = GetAnimBoneTransformWithRoot(name, reset_time - vel_time_step);

		FVector av = QuatToRotationVector(bt.GetRotation()*bt_prev.GetRotation().Inverse()) / vel_time_step;
		FVector lv = (bt.GetTranslation() - bt_prev.GetTranslation()) / vel_time_step;
		SkeletalMesh->GetBodyInstance(name)->SetBodyTransform(bt, ETeleportType::ResetPhysics);
		SkeletalMesh->GetBodyInstance(name)->SetAngularVelocityInRadians(av, false);
		SkeletalMesh->GetBodyInstance(name)->SetLinearVelocity(lv, false);
		//SkeletalMesh->GetBodyInstance(name)->SetAngularVelocityInRadians(FVector::ZeroVector, false);
		//SkeletalMesh->GetBodyInstance(name)->SetLinearVelocity(FVector::ZeroVector, false);
	}
}

void AAndroid::ApplyTorques(double DeltaTime) {
	// UE_LOG(LogHolodeck, Warning, TEXT("AAndroid::ApplyTorques, delta time : %f"), DeltaTime);
	int ComInd = 0;
	double p_gain = 2000000;// 20000000;
	double d_gain = 1000;// 10000;


	torques.Reset();
	forces.Reset();
	torques.Add(FName("pelvis"), FVector(0.0, 0.0, 0.0));
	forces.Add(FName("pelvis"), FVector(0.0, 0.0, 0.0));
	for (int i = 0; i < ModifiedNumBones; i++) {
		torques.Add(ModifiedBoneLists[i], FVector(0.0, 0.0, 0.0));
		forces.Add(ModifiedBoneLists[i], FVector(0.0, 0.0, 0.0));
		// FVector v = SkeletalMesh->GetBodyInstance(ModifiedBoneLists[i])->GetUnrealWorldAngularVelocityInRadians();
		// UE_LOG(LogHolodeck, Warning, TEXT("%s, %f, %f, %f"), *ModifiedBoneLists[i].ToString(), v.X, v.Y, v.Z);
	}
	//UE_LOG(LogTemp, Warning, TEXT("========================================"));
	//UE_LOG(LogTemp, Warning, TEXT("android index : %d"), character_index);
	//UE_LOG(LogTemp, Warning, TEXT("reset signal : %f, %f"), CommandArray[0], CommandArray[1]);
	for (int i = 0; i < ModifiedNumBones; i++) {
		//UE_LOG(LogTemp, Warning, TEXT("%f, %f, %f"), CommandArray[2 + i * 3], CommandArray[2 + i * 3 + 1], CommandArray[2 + i * 3 + 2]);
		FVector action(CommandArray[2 + i * 3], CommandArray[2 + i * 3 + 1], CommandArray[2 + i * 3 + 2]);
		action *= 0.2;
		for (int j = 0; j < 3; j++) {
			if (action[j] > 0.7*PI) {
				action[j] = 0.7*PI;
			}
			if (action[j] < -0.7*PI) {
				action[j] = -0.7*PI;
			}
		}
		applyTorqueByName(ModifiedBoneLists[i], ModifiedBoneParentLists[i], p_gain, d_gain, action);
	}

	//UE_LOG(LogTemp, Warning, TEXT("==========================="));
	//printVector(torques[FName("pelvis")]);
	for (auto& elem : torques) {
		SkeletalMesh->GetBodyInstance(elem.Key)->AddTorqueInRadians(elem.Value);
		/*
		//continue;
		// torque to force and twist
		FVector com_pos = SkeletalMesh->GetBodyInstance(elem.Key)->GetCOMPosition();
		FVector joint_pos = SkeletalMesh->GetBodyInstance(elem.Key)->GetUnrealWorldTransform().GetTranslation();
		FVector torque_normalized = elem.Value;// *SkeletalMesh->GetBodyInstance(elem.Key)->GetBodyMass();
		FVector r = com_pos - joint_pos;
		FVector twist_torque = FVector::DotProduct(torque_normalized, r.GetSafeNormal()) * r.GetSafeNormal();
		FVector swing_torque = torque_normalized - twist_torque;
		FVector swing_force = 0.5*swing_torque.Size() / r.Size() * FVector::CrossProduct(swing_torque.GetSafeNormal(), r.GetSafeNormal());
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
		SkeletalMesh->GetBodyInstance(elem.Key)->AddForceAtPosition(swing_force, 2*com_pos-joint_pos);
		SkeletalMesh->GetBodyInstance(elem.Key)->AddForceAtPosition(-swing_force, joint_pos);
	*/
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
const int AAndroid::NumBodyinstances = 19;

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

