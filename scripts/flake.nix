{
  description = "Python tooling for zenith ground-ops scripts (LoRa param config, etc.)";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs =
    {
      nixpkgs,
      flake-utils,
      ...
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        python = pkgs.python3.withPackages (ps: [
          ps.pymavlink
          ps.pyserial # mavutil needs this to open USB/serial connections (e.g. /dev/ttyACM0)
        ]);
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = [ python ];
        };

        formatter = pkgs.nixfmt-rfc-style;
      }
    );
}
