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
          schemaPath = builtins.getEnv "EYEPIECE_PROTO";
          schema =
            if schemaPath == "" then
              throw "Set EYEPIECE_PROTO to the canonical eyepiece-rpc.proto and use nix build --impure"
            else
              builtins.path {
                path = builtins.toPath schemaPath;
                name = "eyepiece-rpc.proto";
              };
        in
        {
          default =
            pkgs.runCommand "natartos-dist"
              {
                nativeBuildInputs = buildTools pkgs cross ++ [ android.androidsdk pkgs.nanopb pkgs.protobuf ];
                buildInputs = [
                  cross.glibc
                  cross.glibc.static
                ];
                AARCH64_CC = "${cross.stdenv.cc}/bin/${cross.stdenv.cc.targetPrefix}gcc";
                AARCH64_NM = "${cross.stdenv.cc.bintools}/bin/${cross.stdenv.cc.targetPrefix}nm";
                ANDROID_NDK_HOME = "${android.androidsdk}/libexec/android-sdk/ndk-bundle";
                NANOPB_DIR = "${pkgs.nanopb.src}";
                EYEPIECE_PROTO_SOURCE = schema;
                NANOPB_PYTHONPATH = pkgs.python3Packages.makePythonPath [
                  pkgs.nanopb."python-module" pkgs.python3Packages.protobuf
                ];
              }
              ''
                export NATARTOS_DIST_DIR="$out"
                cp "$EYEPIECE_PROTO_SOURCE" "$TMPDIR/eyepiece-rpc.proto"
                export EYEPIECE_PROTO="$TMPDIR/eyepiece-rpc.proto"
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
            packages = buildTools pkgs cross ++ [ android.androidsdk pkgs.nanopb pkgs.protobuf ];
            AARCH64_CC = "${cross.stdenv.cc}/bin/${cross.stdenv.cc.targetPrefix}gcc";
            AARCH64_NM = "${cross.stdenv.cc.bintools}/bin/${cross.stdenv.cc.targetPrefix}nm";
            ANDROID_NDK_HOME = "${android.androidsdk}/libexec/android-sdk/ndk-bundle";
            NANOPB_DIR = "${pkgs.nanopb.src}";
            NANOPB_PYTHONPATH = pkgs.python3Packages.makePythonPath [
              pkgs.nanopb."python-module" pkgs.python3Packages.protobuf
            ];
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
