REM Opens Visual Studio the src folder set as location
REM It sets up the local environment in the VS environment so cmake works!

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
call C:\dev\ros2_jazzy\local_setup.bat
devenv /Edit "./src/"
cmd /k