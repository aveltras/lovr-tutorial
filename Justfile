
set dotenv-load := true
set dotenv-required := true

project_directory := source_directory()
lovr_directory := env('LOVR_DIRECTORY')

android_sdk := env("ANDROID_HOME")
android_ndk := env("ANDROID_NDK_ROOT")
android_abi := env("ANDROID_ABI")
android_native_api_level := env("ANDROID_NATIVE_API_LEVEL")
android_build_tools := env("ANDROID_BUILD_TOOLS_VERSION")
android_stl := env("ANDROID_STL", "c++_shared")
android_assets := env("ANDROID_ASSETS", project_directory + "/src")
android_manifest := env("ANDROID_MANIFEST", lovr_directory + "/etc/AndroidManifest.xml")
android_keystore := env("ANDROID_KEYSTORE", "")

@default:
    just --list

prepare-build-directory directory:
    rm -rf {{ directory }}
    mkdir -p {{ directory }}

[working-directory: "build"]
build: (prepare-build-directory "build")
    cmake {{ lovr_directory }}
    cmake --build .

[working-directory: "android-build"]
apk-build buildtype="Release": (prepare-build-directory "android-build")
    cmake \
      -D CMAKE_TOOLCHAIN_FILE={{ android_ndk }}/build/cmake/android.toolchain.cmake \
      -D ANDROID_SDK={{ android_sdk }} \
      -D ANDROID_ABI={{ android_abi }} \
      -D ANDROID_STL={{ android_stl }} \
      -D ANDROID_NATIVE_API_LEVEL={{ android_native_api_level }} \
      -D ANDROID_BUILD_TOOLS_VERSION={{ android_build_tools }} \
      -D ANDROID_KEYSTORE={{ android_keystore }} \
      -D ANDROID_KEYSTORE_PASS=env:ANDROID_KEYSTORE_PASS \
      -D ANDROID_ASSETS={{ android_assets }} \
      -D ANDROID_MANIFEST={{ android_manifest }} \
      -D CMAKE_BUILD_TYPE={{ buildtype }} \
      {{ lovr_directory }}
    cmake --build .

[working-directory: "android-build"]
apk-install:
    adb install -r lovr.apk

keystore-generate name:
    keytool -genkey -keystore {{ name }}.keystore -alias {{ name }} -keyalg RSA -keysize 2048 -validity 10000





