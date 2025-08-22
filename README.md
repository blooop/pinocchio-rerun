# pinocchio-rerun

This is a renderer project for Pinocchio based on [rerun](https://github.com/rerun-io/rerun).

![solo8](assets/solo8-in-viewer.png)

## Quickstart

All required dependencies (Pinocchio, Assimp, HPP-FCL/coal, EigenPy, example-robot-data, Rerun C++ & Python SDKs, etc.) are resolved automatically via the provided Pixi environment. No manual conda/pip installs are needed.

To view logs you also need the standalone Rerun viewer binary. If it's not on your PATH you can install it separately (see the Rerun [Getting Started](https://github.com/rerun-io/rerun/tree/main#getting-started)). The examples will still run headless and record if the viewer is unavailable.

### Compiling from source

Compiling this from source using CMake:

```bash
mkdir build && cd build
cmake .. -DCMAKE_PREFIX_PATH=<your/prefix/here> -DCMAKE_INSTALL_PREFIX=<your/prefix/here>
cmake --build . --target install
```

When building against conda, you can typically use the environment variables `$CONDA_PREFIX` as your prefix.

### Using Pixi (recommended)

This project ships a `pixi.toml` for fully reproducible, isolated builds via [pixi](https://github.com/prefix-dev/pixi/). All examples/tests execute inside the Pixi environment; 

1. Install pixi (see official docs) or via shell script:
	```bash
	curl -fsSL https://pixi.sh/install.sh | bash
	# then restart your shell so that `pixi` is on PATH
	```
2. Configure & build (configure is cached automatically – reruns are no-ops unless CMakeCache.txt is missing):
	```bash
	pixi run build
	```
3. Run tests:
	```bash
	pixi run test
	```
4. Run examples (each will attempt to spawn/connect to a Rerun viewer):
	```bash
	pixi run example-solo8
	pixi run example-talos
	pixi run example-ur5
	```
5. Install the library (cached with a timestamp; only reinstalls when reconfigured):
	```bash
	pixi run install
	```

Extra tasks:

* `configure` – Run CMake configure step (idempotent).
* `build-debug` – Build a Debug configuration.
* `install` – Installs only when sources reconfigured (timestamp cache).
* `format-check` – Run Ruff on the python sources.
* `clean` – Remove the CMake build directory.

All dependencies (Pinocchio, Assimp, Rerun SDK, etc.) are pulled from `conda-forge` automatically.
