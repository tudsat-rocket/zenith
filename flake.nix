{
  description = "zenith - Embedded flight control firmware for high-powered rockets";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs = {
        nixpkgs.follows = "nixpkgs";
      };
    };
  };
  outputs =
    {
      rust-overlay,
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system}.extend (import rust-overlay);
        rustToolchain = pkgs.rust-bin.fromRustupToolchainFile ./rust-toolchain.toml;

        buildInputs =
          with pkgs;
          [
            # for embedded
            cargo-make
            flip-link # stack overflow protection by via changing memory layout

            # for flashing
            probe-rs-tools
            stlink
            cargo-binutils # provides cargo objcopy to create a binary
            dfu-util # device firmware update
            # pkgs.dfu-programmer
          ]
          ++ [ rustToolchain ];

        # zenith = pkgs.rustPlatform.buildRustPackage {
        #   pname = "zenith";
        #   version = "0.1.0";
        #   description = "Embedded flight control firmware for high-powered rockets";
        #   src = ./.;
        #
        #   cargoLock = {
        #     lockFile = ./Cargo.lock;
        #     allowBuiltinFetchGit = true;
        #   };
        #   inherit buildInputs;
        #
        # };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = buildInputs ++ [ pkgs.just ];
        };

        formatter = pkgs.nixfmt-rfc-style;

        # packages = {
        #   default = zenith;
        # };
      }
    );
}
