git submodule update --init --recursive

pushd external\abseil-cpp
rd /q /s build
mkdir build
cd build
cmake -G "Visual Studio 16 2019" -S .. -B "cmakefiles\Win32\Debug" -DCMAKE_INSTALL_PREFIX="Win32\Debug" -A Win32
cmake -G "Visual Studio 16 2019" -S .. -B "cmakefiles\Win32\Release" -DCMAKE_INSTALL_PREFIX="Win32\Release" -A Win32
cmake -G "Visual Studio 16 2019" -S .. -B "cmakefiles\x64\Debug" -DCMAKE_INSTALL_PREFIX="x64\Debug" -A x64
cmake -G "Visual Studio 16 2019" -S .. -B "cmakefiles\x64\Release" -DCMAKE_INSTALL_PREFIX="x64\Release" -A x64

pushd cmakefiles\Win32\Debug
msbuild INSTALL.vcxproj -property:Configuration=Debug
popd
pushd cmakefiles\Win32\Release
msbuild INSTALL.vcxproj -property:Configuration=Release
popd
pushd cmakefiles\x64\Debug
msbuild INSTALL.vcxproj -property:Configuration=Debug
popd
pushd cmakefiles\x64\Release
msbuild INSTALL.vcxproj -property:Configuration=Release
popd

popd

pushd external\osqp
rd /q /s build
mkdir build
cd build
cmake -G "Visual Studio 16 2019" -S .. -B "Win32" -A Win32
cmake -G "Visual Studio 16 2019" -S .. -B "Win32" -A Win32
cmake -G "Visual Studio 16 2019" -S .. -B "x64" -A x64
cmake -G "Visual Studio 16 2019" -S .. -B "x64" -A x64

pushd Win32
msbuild ALL_BUILD.vcxproj -property:Configuration=Debug
msbuild ALL_BUILD.vcxproj -property:Configuration=Release
popd
pushd x64
msbuild ALL_BUILD.vcxproj -property:Configuration=Debug
msbuild ALL_BUILD.vcxproj -property:Configuration=Release
popd

popd
