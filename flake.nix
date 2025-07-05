{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = { self, nixpkgs, ... }:
    let
      system = "x86_64-linux";
      pkgs = import nixpkgs { inherit system; config = { allowUnfree = true; android_sdk.accept_license = true; }; };
      buildToolsVersion = "34.0.0";
      platformVersion = "29";
      ndkVersion = "26.3.11579264";
      abiVersion = "arm64-v8a";
      androidComposition = pkgs.androidenv.composeAndroidPackages {
        platformVersions = [ platformVersion ];
        buildToolsVersions = [ buildToolsVersion ];
        ndkVersions = [ ndkVersion ];
        abiVersions = [ abiVersion ];
        includeNDK = true;
      };
    in {
      devShells.${system}.default = pkgs.mkShell rec {
        LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath buildInputs;
        ANDROID_HOME = "${androidComposition.androidsdk}/libexec/android-sdk";
        ANDROID_NDK_ROOT = "${ANDROID_HOME}/ndk-bundle";
        ANDROID_ABI = abiVersion;
        ANDROID_NATIVE_API_LEVEL = platformVersion;
        ANDROID_BUILD_TOOLS_VERSION = buildToolsVersion;
        PKG_CONFIG_PATH = pkgs.lib.makeLibraryPath buildInputs;

        LOVR_DEFS = pkgs.fetchFromGitHub {
          owner = "LuaCATS";
          repo = "lovr";
          rev = "3aa8c53b100b9d04e6b26aa00f9926eeebbf5cae";
          hash = "sha256-X5SSta9m5hgFH9HUq+j17R1JuYkk1EXB28+LGoeAgyw=";
        };

        buildInputs = with pkgs; [
          androidComposition.platform-tools
          cmake
          curl
          glslang
          jdk17
          just
          lua-language-server
          pulseaudio
          python3
          vulkan-loader
          xorg.libX11
          xorg.libXcursor
          xorg.libXi
          xorg.libXinerama
          xorg.libXrandr
        ];
      };
    };
}
