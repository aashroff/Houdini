# Houdini Solaris Frustum & Occlusion Culling Prototype

This repository contains a Python prototype for frustum, distance, spherical and occlusion culling designed to run inside Houdini's Solaris (LOPs) context. The primary implementation is in `src/frustum_culling.py` and is intended as a functional prototype and reference — for production use it should be ported to C++ for performance and integration as an HDA.

## Purpose

- Perform multiple culling techniques on USD scene prims in Houdini Solaris:
  - Frustum culling (with optional frustum expansion/padding)
  - Occlusion culling via a software occlusion buffer
  - Distance-based culling
  - Spherical exclusion
  - Support for PointInstancer invisibility lists

This script is useful for reducing render or scene-evaluation workload by disabling or hiding prims that are not needed for a given camera/shot.

## Important notes

- This script is a prototype. It is implemented in Python and uses the USD and Houdini Python APIs. For production, port the logic to C++ (HDK) or a compiled plugin for better performance.
- It is designed to be executed in Houdini Solaris (LOPs) where a stage and camera prim are available.
- The HDA UI (node parameters) must provide specific parameter names used by the script. See the "HDA parameters" section below.

## Installation (Houdini Solaris)

1. In Solaris (LOPs), create a new HDA or Python-based LOP and point its script to `src/frustum_culling.py` (or copy the contents into the node's Python script parameter).
2. Ensure the HDA exposes the following parameters (matching the variable names referenced by the script):

HDA parameters the script expects (names used in the script):

- `camera_path` (string) — Path to the camera prim (e.g. `/root/world/cam`)
- `far_clip_mult` (float) — Multiplier applied to the camera far clip when building the occlusion buffer and expanded frustum
- `padding` (float) — Frustum padding in degrees (applied to FOV)
- `exclusion` (string) — Collection name for explicit exclusion
- `enable_distance_culling` (int/bool) — Toggle distance culling
- `enable_frustum_culling` (int/bool) — Toggle frustum culling
- `culling_method` (string) — One of `deactivate` or `invisible` (how to apply culled state)
- `visualize_frustum` (int/bool) — Toggle drawing a frustum visualization prim
- `enable_spherical_exclusion` (int/bool) — Toggle spherical exclusion around camera
- `enable_occlusion_culling` (int/bool) — Toggle occlusion culling
- `max_occl_culled_prims` (int) — Optional cap for early exit when building occlusion buffer
- `sphere_radius` (float) — Radius for spherical exclusion
- `max_distance` (float) — Maximum distance for distance culling
- `mode` (string) — One of `frame`, `average`, `any` (controls temporal aggregation and how visibility is applied)

Make sure the parameter names and types match exactly, otherwise the script will fail when reading them from `hou.pwd().parent().parm(...)`.

## Usage

- In Solaris, place the node (HDA) that runs the script into your LOP network and point `camera_path` to your shot camera.
- Configure the parameters listed above to match your shot needs.
- The script samples frames between the global `rfstart` and `rfend` context options (or uses a sampling strategy depending on whether Houdini is in interactive mode).

Behavior summary:

- `mode = frame`: visibility changes applied immediately per sampled frame.
- `mode = average`: visibility determined by average visibility across sampled frames.
- `mode = any`: prim is kept visible if visible in any sampled frame.

Occlusion culling: the script builds a software occlusion buffer (numpy-based) at a default resolution (configurable by editing the script variable `occlusion_resolution`) then tests bounding boxes against it. This is a prototype method and may be slow in Python for large scenes.

## API / Code summary (`src/frustum_culling.py`)

- Key functions and responsibilities:
  - `create_occlusion_buffer(stage, camera_prim, resolution, selected_prims, far_clip_mult=1.0, max_occl_culled_prims=None)` — Builds a depth buffer (numpy array) by raycasting bounding boxes. Returns a float32 depth buffer with distances.
  - `is_occluded(bbox, camera_API, occlusion_buffer, occlusion_resolution)` — Tests a world-space bbox centroid against the occlusion buffer.
  - `expand_frustum(frustum, padding, far_clip_multiplier)` — Expands frustum window and far plane.
  - `create_frustum_curves(stage, camera_prim, time_code, padding, far_clip_multiplier)` — Adds a visualization `BasisCurves` prim to the stage for debugging.
  - `is_within_sphere(prim_bbox, camera_origin, sphere_radius)` — Spherical exclusion helper.
  - `distance_based_culling(prim, camera_API, max_distance, bbox_cache, time_code, culling_method)` — Applies simple distance-based culling.

The script also handles PointInstancer-specific logic (computing invisible id lists, averaging across frames) and writes visibility attributes or deactivates prims depending on `culling_method`.

## Development / Performance

- This prototype relies on Python, USD, and Houdini APIs and will be CPU-bound and slow for large scenes. For production:
  - Port compute-heavy parts (occlusion buffer generation, ray/bbox intersection, frustum tests) to C++ using the Houdini HDK and a compiled HDA or LOP plugin.
  - Consider multi-threading or GPU-based occlusion (rasterization) to accelerate occlusion tests.

## Quick start (example)

1. Create an HDA LOP and paste the script contents from `src/frustum_culling.py` into the node's Python script parameter.
2. Create the parameters shown in the Installation section and wire them up to UI controls.
3. In Solaris, set `camera_path` to your camera prim and enable the desired culling toggles.

## License
- MIT 

## Next steps / TODOs

- Port performance-critical code to C++ (HDK) and provide a compiled LOP for production.
- Add unit tests for geometric helpers (frustum intersection, ray-box intersection).
- Make occlusion buffer resolution and sampling strategy configurable via HDA parameters.
- Add logging and error handling for edge cases encountered in large USD stages.
