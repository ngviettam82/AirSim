# CI/CD

This repository includes both GitLab CI/CD and GitHub Actions workflow files. The current `origin` remote is GitLab, so `.gitlab-ci.yml` is the pipeline that runs after pushes to the project remote.

The automation has two levels:

1. Hosted or Docker CI validates text hygiene, Python packaging, documentation, and Linux AirLib builds.
2. Windows runner jobs can build AirLib and package the Unreal plugin with Unreal Engine 5.5.

## GitLab CI

The GitLab pipeline runs for merge requests, pushes to the default branch, tags, and manually started web pipelines. It performs:

- repository text checks for unresolved merge markers and mixed line endings inside individual files;
- GitHub Actions and GitLab CI YAML parsing;
- Python bytecode compilation for `PythonClient`;
- Python package build from `PythonClient/pyproject.toml`;
- MkDocs strict documentation build;
- AirLib builds on Ubuntu 22.04 and Ubuntu 24.04.

The repository intentionally preserves both LF and CRLF files through `.gitattributes`. CI only fails a file when that single file mixes line endings.

## Windows Jobs

The GitLab `build_windows` job is manual by default, or automatic when the pipeline variable `RUN_WINDOWS_CI` is set to `true`. It requires a Windows runner tagged with `windows` and `vs2022`.

The `package_unreal_windows` job is manual for tags and manually started pipelines. It requires a Windows runner tagged with `windows` and `unreal`.

Runner requirements:

- Windows x64 runner;
- Unreal Engine 5.5 installed, by default at `C:\Program Files\Epic Games\UE_5.5`;
- Visual Studio 2022 with Desktop Development with C++;
- Git LFS enabled if release assets are needed.

The Unreal package job runs:

1. `build.cmd --no-full-poly-car --Release`
2. `Unreal/Environments/Blocks/update_from_git.bat`
3. `Build.bat BlocksEditor Win64 Development`
4. `RunUAT.bat BuildPlugin`
5. artifact upload as `AirSimPlugin-Win64.zip`

## GitHub Actions

The `.github/workflows` files mirror the same checks for GitHub-hosted usage. They are useful if the repository is mirrored to GitHub, but GitLab will not execute them.

## Local Checks

Before pushing CI/CD changes, run:

```powershell
python tools\ci\check_repo_text.py
python -m compileall -q PythonClient tools\ci
python -m build PythonClient
python -m mkdocs build --strict
build.cmd --no-full-poly-car --Release
```

Use `C:\Program Files\Epic Games\UE_5.5` or override `UE_ROOT` in the GitLab pipeline variables if Unreal is installed somewhere else.
