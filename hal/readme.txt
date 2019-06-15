Key rule:  The hal directory (and it's subdirectories) is the ONLY directory of the WPI Library Source that may be modified.

Following this rule ensures that the other WPI Library components (wpilibc, ntcore, cscore, etc) may be used without modification.

Notes:

- CMakeLists.txt:  This file is heavily modified from the original WPI library version.
- build.gradle:  This file is from the original WPI Library version, and is not used.
- simjni.gradle:  This file is from the original WPI Library version, and is not used.
- ./gradle subdirectory:  This directory contains gradle scripts to generate distributable artifacts in maven publication format.
- ./dist subdirectory:  This directory contains addtional scripts to distribute artifacts to remote maven repositories.
- ./src/main subdirectory:  This directory and all sub-directories contain sources (some heavily modified from the original WPI library version, some new).

To build, see the scripts/instructions in the KauaiLabs directory, which is a sibling of this (hal) directory.