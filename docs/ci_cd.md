# CI/CD

This repository uses GitHub Actions for two levels of automation:

1. Hosted CI validates text hygiene, Python packaging, documentation, and AirLib builds on Linux and Windows.
2. Self-hosted Unreal packaging builds the Blocks editor target and packages the AirSim plugin with Unreal Engine 5.5.

## Hosted CI

The `CI` workflow runs on pull requests, pushes to `main`, and manual dispatch. It performs:

- repository text checks for unresolved merge markers and mixed line endings inside individual files;
- GitHub Actions YAML parsing;
- Python bytecode compilation for `PythonClient`;
- Python package build from `PythonClient/pyproject.toml`;
- MkDocs strict documentation build;
- AirLib builds on Ubuntu 22.04, Ubuntu 24.04, and Windows Server 2022.

The repository intentionally preserves both LF and CRLF files through `.gitattributes`. CI only fails a file when that single file mixes line endings.

## Unreal Packaging

The `Unreal Plugin Package` workflow is the CD path. It requires a self-hosted Windows runner because GitHub-hosted runners do not include Unreal Engine.

Runner requirements:

- Windows x64 self-hosted runner with the labels `self-hosted`, `Windows`, `X64`, and `unreal`;
- Unreal Engine 5.5 installed, by default at `C:\Program Files\Epic Games\UE_5.5`;
- Visual Studio 2022 with Desktop Development with C++;
- Git LFS enabled if release assets are needed.

The workflow runs:

1. `build.cmd --no-full-poly-car --Release`
2. `Unreal/Environments/Blocks/update_from_git.bat`
3. `Build.bat BlocksEditor Win64 Development`
4. `RunUAT.bat BuildPlugin`
5. artifact upload as `AirSimPlugin-Win64.zip`

When the workflow is triggered by a `v*` tag, the package is also attached to the GitHub release for that tag.
