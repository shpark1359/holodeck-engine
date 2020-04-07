// MIT License (c) 2019 BYU PCCL see LICENSE file

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Animation/AnimSequence.h"
#include "ReferenceMotion.generated.h"

UCLASS()
class HOLODECK_API AReferenceMotion : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AReferenceMotion();

	UAnimSequence* IdleAnim;
	USkeleton* skeleton;

	const static FName BodyInstanceNames[];
	const static FName ModifiedBoneLists[];
	const static FName ModifiedBoneParentLists[];
	const static int NumBodyinstances;
	const static int ModifiedNumBones;

	UPROPERTY(BlueprintReadWrite, Category = AndroidMesh)
		USkeletalMeshComponent* SkeletalMesh;


	UPROPERTY(BlueprintReadWrite, EditAnyWhere)
		float cur_time;
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	FTransform GetAnimBoneTransform(FName b_name, float time);
	FTransform GetAnimBoneTransformWithRoot(FName b_name, float time);
	FTransform GetAnimBoneTransformWithRoot(FName b_name);
	FTransform GetAnimBoneTransformWithRootNext(FName b_name);

	TArray<TMap<FName, FTransform>> animation_data;
	bool is_animation_loaded = false;
	float time_step;

	FVector root_offset;
};
