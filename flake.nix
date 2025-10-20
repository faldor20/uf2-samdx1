{
  description = "A basic flake with a shell";
  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  inputs.systems.url = "github:nix-systems/default";
  inputs.flake-utils = {
    url = "github:numtide/flake-utils";
    inputs.systems.follows = "systems";
  };

  outputs =
    { nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell { 
          packages = [
            pkgs.bashInteractive
            pkgs.gnumake
            pkgs.emscripten
            pkgs.gcc-arm-embedded
            pkgs.compiledb
            pkgs.glibc_multi
            pkgs.openocd
            pkgs.pyocd
          ];
          shellHook = ''
            export EM_CACHE=$(pwd)/.emscripten_cache/
            export USE_ST_LPS28DFW=1
            export BOARD=sensorwatch_pro
            export DISPLAY=CUSTOM
          '';
        };
      }
    );
}
