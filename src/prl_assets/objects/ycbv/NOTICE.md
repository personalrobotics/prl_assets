# YCB-Video collection: sources and notices

The package's MIT license covers its code. It does not relicense the object scans in
this directory, which keep the terms of their sources below.

## YCB Object and Model Set

The meshes and textures in this collection are derived from scans in the YCB Object
and Model Set, published under the Creative Commons Attribution 4.0 International
license (CC BY 4.0, <https://creativecommons.org/licenses/by/4.0/>).

> B. Calli, A. Walsman, A. Singh, S. Srinivasa, P. Abbeel, and A. M. Dollar,
> "Benchmarking in Manipulation Research: Using the Yale-CMU-Berkeley Object and Model
> Set," IEEE Robotics & Automation Magazine, 22(3), 2015.
> Source: <https://ycb-benchmarks.s3.amazonaws.com/index.html>

Changes made here: the four pre-existing meshes are remeshed and expressed in metres; some are rotated
into the asset frame declared in each `meta.yaml` (`frames.T_asset_bop`); textures are
resampled to 1024 × 1024; collision geometry, mass and contact parameters are added.

## YCB-Video and the BOP release

Object identities (`dataset.ycbv.bop_id`), reference geometry, and symmetry metadata
follow the YCB-Video dataset as released by the BOP benchmark, whose distribution
declares the MIT license.

> Y. Xiang, T. Schmidt, V. Narayanan, and D. Fox, "PoseCNN: A Convolutional Neural
> Network for 6D Object Pose Estimation in Cluttered Scenes," RSS 2018.
> Source: <https://github.com/yuxng/YCB_Video_toolbox>

> BOP: Benchmark for 6D Object Pose Estimation, `ycbv_models.zip` at
> <https://huggingface.co/datasets/bop-benchmark/ycbv/tree/5c2c4aa229800355648cd268040aa814f8dc94f0>,
> SHA-256 `1e517a9b513cadb8eaf406ac3ce3c5f1d6fc4a80161c866b4e2a73d13de5e554`.

The SHA-256 of each reference model file used is recorded in `docs/ycbv_audit.json`.

New imports retain the BOP visual vertices and UVs, converted from millimetres to metres,
with 1024 by 1024 textures and separate convex collision pieces. Reference YCB masses
come from Calli et al. (2015), Table 2; inertia and friction are explicitly estimated.
