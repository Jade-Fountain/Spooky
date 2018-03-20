/**
  * 
  */
#pragma once

#include "Spooky.h"

#include "WriteToFile.generated.h"

 UCLASS()
 class UWriteToFile : public UBlueprintFunctionLibrary
 {
     GENERATED_UCLASS_BODY()
     
 public:
     UFUNCTION(BlueprintCallable, meta = (HidePin = "WorldContextObject", DefaultToSelf = "WorldContextObject", DisplayName = "File-IO"), Category = "WriteToFile")
         static bool FileIO__SaveStringTextToFile(FString SaveDirectory, FString fileName, FString SaveText, bool AllowOverWriting);
     
 };
 