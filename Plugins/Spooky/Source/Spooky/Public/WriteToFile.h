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
     UFUNCTION(BlueprintCallable, meta = (HidePin = "WorldContextObject", DefaultToSelf = "WorldContextObject", DisplayName = "AppendStringToTextFile"), Category = "WriteToFile")
         static bool FileIO__AppendStringTextToFile(FString SaveDirectory, FString fileName, FString SaveText, bool AllowOverWriting);
     
 };
 