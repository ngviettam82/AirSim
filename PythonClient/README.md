# Python API for AirSim

Python client for **this** AirSim repository (RPC over the sim API). It is the
modified client that matches the plugins and APIs in this tree — not the
Cosys-Lab or Microsoft PyPI packages.

Import name:

```python
import airsim
```

## Dependencies

`numpy` and `rpc-msgpack` are pulled in automatically when you install from
`PythonClient`. You can also install them yourself:

```bash
pip install numpy rpc-msgpack
```

## Install from this repository (required)

Do **not** use `pip install cosysairsim` or other PyPI AirSim packages with this
fork. Those wheels do not match our API surface.

From a clone of this repo:

```bash
cd PythonClient
pip install .
```

Editable install (recommended while developing APIs):

```bash
cd PythonClient
pip install -e .
```

Then:

```python
import airsim
client = airsim.MultirotorClient()
client.confirmConnection()
```

## Running examples without installing

Example scripts under `PythonClient/car`, `multirotor`, and so on import
`setup_path` first so the in-tree `airsim` package is preferred:

```bash
cd PythonClient/multirotor
python hello_drone.py
```

## More info

- API guide: [docs/apis.md](../docs/apis.md)
- Repository: https://github.com/ngviettam82/Airsim
