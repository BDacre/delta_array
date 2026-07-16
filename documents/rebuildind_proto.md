# Rebuilding


When changes have been made to the `.proto` file, we must rebuild all places that use it to ensure version match.

With the venv activated (`source .venv/bin/activate`), the rebuild after editing a `.proto` is a two-step regeneration — one for firmware C, one for Python:

**Please check the version landscape for details**
[[Protobuff delta version landscape]]

## Regenerate the firmware C output (nanopb)

within the bundled  nanopb library in plaformio — `microcontroller/FINAL_delta_array_12motor/.pio/libdeps/feather_m0/Nanopb/library.json` declares:

```json
"build": {
  "extraScript": "generator/platformio_generator.py",
  ...
}
```

That means **platformio auto-regenerates `.pb.c/.pb.h` from  `.proto` at build time**, using the generator it shipped with. Version match is automatic. 

either run *pio run* or *press build in vs code*

**No need to run `nanopb_generator.py` manually for the firmware at all. Doing so will cause a version mismatch**

IF a version mismatch occurs and you can not build in plaformio, you can rerun using the generator within `.pio/libdeps` using the following commands. **Ensure it uses the venvs python**

```
cd /home/bailey/Repos/DELTA_ARRAY/delta_array/delta_array/microcontroller/delta_array_12_motor/src

/home/bailey/Repos/DELTA_ARRAY/delta_array/delta_array/.venv/bin/python ../.pio/libdeps/feather_m0/Nanopb/generator/nanopb_generator.py delta_array.proto
```

## Regenerate the Python bindings

```bash
cd microcontroller/delta_array_12motor/src
python -m grpc_tools.protoc \
    --proto_path=. \
    --python_out=../../../python/delta_control \
    delta_array.proto
```

### Piece by piece

**`python -m grpc_tools.protoc`** The `-m` flag tells Python "import this module and run it as a script." So Python loads `grpc_tools/protoc.py` from the venv's `site-packages`. That module is a small wrapper — when invoked, it locates the **prebuilt `protoc` binary that grpcio-tools ships inside its own package directory** (`.venv/lib/python3.10/site-packages/grpc_tools/protoc`, version 31.1) and `exec`s it with the remaining arguments. It's just a portable way to invoke that bundled compiler without needing it on PATH.

**`--proto_path=.`** (sometimes spelled `-I`) Tells protoc where to look when resolving `.proto` files — both the input file and any `import` statements inside it. `.` means "current directory." Since you `cd`'d into `microcontroller/FINAL_delta_array_12motor/src/` first, that's where it'll look. If `delta_array.proto` had a line like `import "google/protobuf/timestamp.proto"`, this is the search path used to find that.

**`--python_out=../../../python/delta_control`** Tells protoc: "for each input proto, generate Python output and write it into this directory." Relative to your current working directory (`src/`), that path resolves to `python/delta_control/` at the project root. Protoc creates one `_pb2.py` file per input proto — so `delta_array.proto` → `delta_array_pb2.py` written into `python/delta_control/`.

**`delta_array.proto`** The input file. Resolved using `--proto_path`, so it's `./delta_array.proto`.
