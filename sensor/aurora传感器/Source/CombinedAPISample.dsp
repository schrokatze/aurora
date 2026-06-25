# Microsoft Developer Studio Project File - Name="NDI CombinedAPI Sample" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Application" 0x0101

CFG=NDI CombinedAPI Sample - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "CombinedAPISample.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "CombinedAPISample.mak" CFG="NDI CombinedAPI Sample - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "NDI CombinedAPI Sample - Win32 Release" (based on "Win32 (x86) Application")
!MESSAGE "NDI CombinedAPI Sample - Win32 Debug" (based on "Win32 (x86) Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "NDI CombinedAPI Sample - Win32 Release"

# PROP BASE Use_MFC 6
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 6
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /Yu"stdafx.h" /FD /c
# ADD CPP /nologo /MD /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /Yu"stdafx.h" /FD /c
# ADD BASE MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "NDEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "NDEBUG" /d "_AFXDLL"
# ADD RSC /l 0x409 /d "NDEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 /nologo /subsystem:windows /machine:I386
# ADD LINK32 /nologo /subsystem:windows /machine:I386 /out:"Release/NDI CombinedAPI Sample.exe"

!ELSEIF  "$(CFG)" == "NDI CombinedAPI Sample - Win32 Debug"

# PROP BASE Use_MFC 6
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 6
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_AFXDLL" /Yu"stdafx.h" /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_WINDOWS" /D "_AFXDLL" /D "_MBCS" /Yu"stdafx.h" /FD /GZ /c
# ADD BASE MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD MTL /nologo /D "_DEBUG" /mktyplib203 /win32
# ADD BASE RSC /l 0x409 /d "_DEBUG" /d "_AFXDLL"
# ADD RSC /l 0x409 /d "_DEBUG" /d "_AFXDLL"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 /nologo /subsystem:windows /debug /machine:I386 /pdbtype:sept
# ADD LINK32 /nologo /subsystem:windows /debug /machine:I386 /out:"Debug/NDI CombinedAPI Sample.exe" /pdbtype:sept

!ENDIF 

# Begin Target

# Name "NDI CombinedAPI Sample - Win32 Release"
# Name "NDI CombinedAPI Sample - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\CombinedAPISample.cpp
# End Source File
# Begin Source File

SOURCE=.\CombinedAPISample.rc
# End Source File
# Begin Source File

SOURCE=.\CombinedAPISampleDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\Comm32.cpp
# SUBTRACT CPP /YX /Yc /Yu
# End Source File
# Begin Source File

SOURCE=.\CommandConstruction.cpp
# SUBTRACT CPP /YX /Yc /Yu
# End Source File
# Begin Source File

SOURCE=.\CommandHandling.cpp
# End Source File
# Begin Source File

SOURCE=.\ComPortSettings.cpp
# End Source File
# Begin Source File

SOURCE=.\ComPortTimeout.cpp
# End Source File
# Begin Source File

SOURCE=.\Conversions.cpp
# SUBTRACT CPP /YX /Yc /Yu
# End Source File
# Begin Source File

SOURCE=.\IlluminatorFiringRate.cpp
# End Source File
# Begin Source File

SOURCE=.\INIFileRW.cpp
# SUBTRACT CPP /YX /Yc /Yu
# End Source File
# Begin Source File

SOURCE=.\NewAlertFlagsDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\ProgramOptions.cpp
# End Source File
# Begin Source File

SOURCE=.\ROMFileDlg.cpp
# End Source File
# Begin Source File

SOURCE=.\StdAfx.cpp
# ADD CPP /Yc"stdafx.h"
# End Source File
# Begin Source File

SOURCE=.\SystemCRC.cpp
# SUBTRACT CPP /YX /Yc /Yu
# End Source File
# Begin Source File

SOURCE=.\SystemFeaturesDlg.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\APIStructures.h
# End Source File
# Begin Source File

SOURCE=.\CombinedAPISample.h
# End Source File
# Begin Source File

SOURCE=.\CombinedAPISampleDlg.h
# End Source File
# Begin Source File

SOURCE=.\Comm32.h
# End Source File
# Begin Source File

SOURCE=.\CommandHandling.h
# End Source File
# Begin Source File

SOURCE=.\ComPortSettings.h
# End Source File
# Begin Source File

SOURCE=.\ComPortTimeout.h
# End Source File
# Begin Source File

SOURCE=.\Conversions.h
# End Source File
# Begin Source File

SOURCE=.\IlluminatorFiringRate.h
# End Source File
# Begin Source File

SOURCE=.\INIFileRW.h
# End Source File
# Begin Source File

SOURCE=.\NewAlertFlagsDlg.h
# End Source File
# Begin Source File

SOURCE=.\ProgramOptions.h
# End Source File
# Begin Source File

SOURCE=.\Resource.h
# End Source File
# Begin Source File

SOURCE=.\ROMFileDlg.h
# End Source File
# Begin Source File

SOURCE=.\StdAfx.h
# End Source File
# Begin Source File

SOURCE=.\SystemFeaturesDlg.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# Begin Source File

SOURCE=.\res\bitmap1.bmp
# End Source File
# Begin Source File

SOURCE=.\res\bmp00001.bmp
# End Source File
# Begin Source File

SOURCE=.\res\CombinedAPISample.ico
# End Source File
# Begin Source File

SOURCE=.\res\CombinedAPISample.rc2
# End Source File
# Begin Source File

SOURCE=.\res\timeout.ico
# End Source File
# End Group
# Begin Source File

SOURCE=.\Documents\ReadMe.txt
# End Source File
# End Target
# End Project
