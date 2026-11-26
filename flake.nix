{
  description = "natartos AArch64 build environment";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

  outputs =
    { self, nixpkgs }:
    let
      systems = [
        "aarch64-darwin"
        "aarch64-linux"
        "x86_64-linux"
      ];
      forEachSystem = nixpkgs.lib.genAttrs systems;
      makePkgs =
        system:
        import nixpkgs {
          inherit system;
          config = {
            allowUnfree = true;
            android_sdk.accept_license = true;
          };
        };
      makeCross = system: (makePkgs system).pkgsCross.aarch64-multiplatform;
      buildTools =
        pkgs: cross: with pkgs; [
          bash
          coreutils
          gawk
          gnugrep
          gnused
          python3
          cross.stdenv.cc
          cross.stdenv.cc.bintools
        ];
    in
    {
      packages = forEachSystem (
        system:
        let
          pkgs = makePkgs system;
          cross = makeCross system;
          android = pkgs.androidenv.composeAndroidPackages {
            includeNDK = true;
            platformVersions = [ "28" ];
          };
        in
        {
          default =
            pkgs.runCommand "natartos-dist"
              {
                nativeBuildInputs = buildTools pkgs cross ++ [ android.androidsdk ];
                buildInputs = [
                  cross.glibc
                  cross.glibc.static
                ];
                AARCH64_CC = "${cross.stdenv.cc}/bin/${cross.stdenv.cc.targetPrefix}gcc";
                AARCH64_NM = "${cross.stdenv.cc.bintools}/bin/${cross.stdenv.cc.targetPrefix}nm";
                ANDROID_NDK_HOME = "${android.androidsdk}/libexec/android-sdk/ndk-bundle";
              }
              ''
                export NATARTOS_DIST_DIR="$out"
                bash ${./.}/scripts/build.sh
              '';
        }
      );

      devShells = forEachSystem (
        system:
        let
          pkgs = makePkgs system;
          cross = makeCross system;
          android = pkgs.androidenv.composeAndroidPackages {
            includeNDK = true;
            platformVersions = [ "28" ];
          };
          defaultShell = pkgs.mkShell {
            packages = buildTools pkgs cross ++ [ android.androidsdk ];
            AARCH64_CC = "${cross.stdenv.cc}/bin/${cross.stdenv.cc.targetPrefix}gcc";
            AARCH64_NM = "${cross.stdenv.cc.bintools}/bin/${cross.stdenv.cc.targetPrefix}nm";
            ANDROID_NDK_HOME = "${android.androidsdk}/libexec/android-sdk/ndk-bundle";
            shellHook = ''
              echo "natartos: Nix AArch64 and Android NDK build ready; run ./scripts/build.sh"
            '';
          };
        in
        {
          default = defaultShell;
          android = defaultShell;
        }
      );
    };
}
