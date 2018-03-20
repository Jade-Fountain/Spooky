// Not a part of Spooky.
//Source: https://answers.unrealengine.com/questions/94938/save-a-text-file-using-blueprints.html
 
#include "Spooky.h"
#include "WriteToFile.h"
 
 UWriteToFile::UWriteToFile(const class FObjectInitializer& PCIP)
     :Super(PCIP)
 {
 
 }
 
 bool UWriteToFile::FileIO__SaveStringTextToFile(FString SaveDirectory, FString fileName, FString SaveText, bool AllowOverWriting)
 {
     FString path;
     path = FPaths::GameDir();
     path += "/my_data";
 
     if (!FPlatformFileManager::Get().GetPlatformFile().DirectoryExists(*path))
     {
         FPlatformFileManager::Get().GetPlatformFile().CreateDirectory(*path);
         if (!FPlatformFileManager::Get().GetPlatformFile().DirectoryExists(*path))
         {
             return false;
         }
     }
 
     path += "/";
     path += fileName;
 
     if (!AllowOverWriting)
     {
         if (FPlatformFileManager::Get().GetPlatformFile().FileExists(*path))
         {
             return false;
         }
     }
     
     return FFileHelper::SaveStringToFile(SaveText, *path);
 }