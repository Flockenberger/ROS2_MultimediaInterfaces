REM Written by Florian Wagner 11907095
REM This opens a visual studio promt and runs the colcon build and calls setup.bat
REM This assumes that VS2022 is installed in the given location below, similarly
REM It assumes that you have your ros2 installed in c:\dev\
REM Simply change the locations below if needed!

call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
call C:\dev\ros2_jazzy\local_setup.bat
colcon build --merge-install
call install\setup.bat
cmd /k