#include "Holodeck.h"
#include "CustomSensor.h"
#include "Utils.h"

UCustomSensor::UCustomSensor() {
	PrimaryComponentTick.bCanEverTick = true;
	SensorName = "CustomSensor";
}

void UCustomSensor::InitializeSensor() {

	UE_LOG(LogHolodeck, Warning, TEXT("UCustomSensor::InitializeSensor()"));
	this->Parent = Cast<AAndroid>(this->GetOwner());

	if (!this->Parent->IsA(AAndroid::StaticClass())) {
		UE_LOG(LogHolodeck, Fatal, TEXT("Error: Tried to use UCustomSensor with unknown agent type."));
	}

	this->NumJoints = 18;
	this->NumEEs = 5;
	this->StateSize = this->NumJoints * 3 + this->NumEEs * 3 + this->NumEEs * 3;

	this->RewardSize = 5;
	this->TotalSize = this->StateSize + this->RewardSize;

	this->EEList.Add(FName("head"));
	this->EEList.Add(FName("hand_l"));
	this->EEList.Add(FName("hand_r"));
	this->EEList.Add(FName("foot_l"));
	this->EEList.Add(FName("foot_r"));
	
	TArray<USkeletalMeshComponent*> Components;
	this->Parent->GetComponents<USkeletalMeshComponent>(Components);
	this->SkeletalMeshComponent = Components[0];

	Super::InitializeSensor();
}

int UCustomSensor::GetNumItems() {
	return this->TotalSize;
}

void UCustomSensor::TickSensorComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) {
	this->GetState();
	this->GetReward();
}

void UCustomSensor::GetState() {
	float* FloatBuffer = static_cast<float*>(Buffer);
	int index = 0;
	// joint angles
	for (int i = 0; i < this->NumJoints; i++) {
		FName b_name = AAndroid::ModifiedBoneLists[i];
		FVector angle = this->Parent->getJointAngle(b_name);
		FloatBuffer[index] = angle[0];
		FloatBuffer[index+1] = angle[1];
		FloatBuffer[index+2] = angle[2];
		index += 3;
	}
	// ee positions
	FTransform root_transform = this->SkeletalMeshComponent->GetBodyInstance(FName("pelvis"))->GetUnrealWorldTransform();
	for (int i = 0; i < this->NumEEs; i++) {
		FName b_name = this->EEList[i];
		FTransform tf = this->SkeletalMeshComponent->GetBodyInstance(b_name)->GetUnrealWorldTransform();
		FTransform local_tf = tf * root_transform.Inverse();
		FVector translation = local_tf.GetTranslation();
		FloatBuffer[index] = translation[0];
		FloatBuffer[index + 1] = translation[1];
		FloatBuffer[index + 2] = translation[2];
		index += 3;		
	}
	// future ee positions
	for (int i = 0; i < this->NumEEs; i++) {
		FName b_name = this->EEList[i];
		FTransform tf = this->Parent->GetAnimBoneTransformWithRootNext(b_name);
		FTransform local_tf = tf * root_transform.Inverse();
		FVector translation = local_tf.GetTranslation();
		FloatBuffer[index] = translation[0];
		FloatBuffer[index + 1] = translation[1];
		FloatBuffer[index + 2] = translation[2];
		index += 3;
	}
}

void UCustomSensor::GetReward() {
	float sig_p = 0.5;
	float sig_v = 1.0;
	float sig_com = 0.2;
	float sig_ee = 0.2;

	// p reward
	float rew_p = 0;
	for (int i = 0; i < this->NumJoints; i++) {
		FName b_name = AAndroid::ModifiedBoneLists[i];
		FVector angle = this->Parent->getJointAngle(b_name);
		FVector target_angle = this->Parent->getReferenceJointAngle(b_name);
		rew_p += QuatToRotationVector(RotationVectorToQuat(angle).Inverse()*RotationVectorToQuat(target_angle)).SizeSquared();
	}
	rew_p /= (this->NumJoints * 3);
	rew_p = exp(-rew_p / (sig_p * sig_p));

	// v reward

	UE_LOG(LogTemp, Warning, TEXT("=========================="));
	float rew_v = 0;
	for (int i = 0; i < this->NumJoints; i++) {
		FName b_name = AAndroid::ModifiedBoneLists[i];
		FVector av = this->Parent->getJointAngularVelocity(b_name);
		FVector target_av = this->Parent->getReferenceJointAngularVelocity(b_name);
		rew_v += (av - target_av).SizeSquared();
		printVector(av);
		printVector(target_av);

	}
	float rew_v_t = rew_v;
	rew_v /= (this->NumJoints * 3);
	rew_v = exp(-rew_v / (sig_v * sig_v));

	// com reward
	float rew_com = 0;
	FTransform root_transform = this->SkeletalMeshComponent->GetBodyInstance(FName("pelvis"))->GetUnrealWorldTransform();
	FTransform target_root_transform = this->Parent->GetAnimBoneTransformWithRoot(FName("pelvis"));
	rew_com = (root_transform.GetTranslation() - target_root_transform.GetTranslation()).SizeSquared();
	rew_com /= 3;
	rew_com = exp(-rew_com / (sig_com * sig_com));

	// ee reward
	float rew_ee = 0;
	for (int i = 0; i < this->NumEEs; i++) {
		FName b_name = this->EEList[i];
		FTransform tf = this->SkeletalMeshComponent->GetBodyInstance(b_name)->GetUnrealWorldTransform();
		FTransform target_tf = this->Parent->GetAnimBoneTransformWithRootNext(b_name);
		rew_ee += (tf.GetTranslation() - target_tf.GetTranslation()).SizeSquared();
	}
	rew_ee /= (this->NumEEs * 3);
	rew_ee = exp(-rew_ee / (sig_ee*sig_ee));

	float rew = rew_p * rew_v * rew_com * rew_ee;

	float* FloatBuffer = static_cast<float*>(Buffer);
	int index = this->StateSize;
	FloatBuffer[index + 0] = rew;
	FloatBuffer[index + 1] = rew_p;
	FloatBuffer[index + 2] = rew_v;
	FloatBuffer[index + 3] = rew_com;
	FloatBuffer[index + 4] = rew_ee;

	// UE_LOG(LogTemp, Warning, TEXT("GetReward : %f, %f, %f, %f, %f"), rew, rew_p, rew_v_t, rew_com, rew_ee);

}