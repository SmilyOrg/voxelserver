@echo off
mkdir build
mkdir artifacts
cd build
cmake .. -G "Visual Studio 14 2015 Win64"
cmake --build . --config RelWithDebInfo
copy RelWithDebInfo\voxelserver.exe ..\artifacts\voxelserver.exe
copy RelWithDebInfo\voxelserver.pdb ..\artifacts\voxelserver.pdb
cd ..
robocopy www artifacts\www /MIR /XD node_modules semantic
robocopy www\semantic\dist artifacts\www\semantic\dist /MIR
echo ----------------------
echo Compiled to artifacts/
echo ----------------------