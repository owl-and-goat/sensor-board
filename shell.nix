{
  pkgs ? import <nixpkgs> { },
}:

with pkgs;

mkShell {
  buildInputs = [
    just
    dfu-util
    probe-rs-tools
  ];

  PROBE_RS_CHIP = "stm32wb55cg";
}
