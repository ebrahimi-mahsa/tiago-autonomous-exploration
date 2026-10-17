# Packaging notes

Source: the user-supplied tiago_task1_for_github.tar.gz archive.

The original exploration and arm-tucking scripts, launch files, map PGM and YAML are preserved byte for byte. Package metadata and installation rules receive two small packaging corrections documented in NOTICE. RViz configurations are included as supplied; they are not asserted to encode the exact final recording layout.

The example-world package and other robot dependencies were not in the archive. This repository therefore requires the existing TIAGo dependency environment. No clean-machine build was performed.

The sample YAML references the PGM by relative basename, so they can be kept together in maps/ without editing the YAML.

The map preview is derived from the original PGM with nearest-neighbour enlargement. The demo is edited from the user's actual simulation recording; it includes no generated simulation imagery.
