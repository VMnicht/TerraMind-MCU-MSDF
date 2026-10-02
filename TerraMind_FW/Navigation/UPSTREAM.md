# KF-GINS H7 port provenance

The files in `core/common/` and `core/kf-gins/` derive from
[i2Nav-WHU/KF-GINS](https://github.com/i2Nav-WHU/KF-GINS), pinned at
`8291a93e49de513fe9d21f819500d39082ded611` in this repository's `KF-GINS` submodule.
Copyright (C) 2022 i2Nav Group, Wuhan University; original authors include
Liqiang Wang and Hailiang Tang. The derived core remains GPL-3.0-or-later;
the full license is in `../../KF-GINS/LICENSE`.

The original submodule is not modified. Its executable and source are the
numerical reference, not a dependency on a host process at runtime.

Port changes:

- Isolate symbols in `tmnav`; prefix header guards and remove desktop streams.
- Preserve 21 error states, 18 driving noises, double precision, mechanization,
  all error model blocks, GNSS lever-arm Jacobian, time interpolation, feedback,
  first-order transition and the original process-noise discretization.
- Replace dynamic Eigen matrices with fixed shapes. Put large products in an
  engine-owned workspace and split chained products without changing equations.
  Small expression temporaries and Eigen kernel scratch still use the task stack;
  stack sizing must be measured on the actual compiler/board.
- Keep Joseph covariance update and full cross-covariances; no state reduction,
  covariance decimation, float conversion, or fast-math approximation.
- Initialize the empty GNSS slot; skip update classification when it is empty.
  This prevents the upstream `-1` sentinel from matching a legitimate time axis.
- Replace process termination on bad covariance with a latched fault. Also
  reject nonfinite state/covariance and singular innovation matrices. Recovery
  requires explicit reinitialization, never an unreported algorithm reset.
- Remove console-only Euler singularity warnings; preserve their numerical
  branches. No navigation code prints to or transmits on a firmware UART.
- Use `EIGEN_NO_MALLOC` with assertions enabled during host verification.
  The C boundary also allocates nothing: the caller owns aligned context memory.

`tests/navigation/reference.cpp` links only the unmodified upstream core in a
separate executable. `nav_selftest` compares every navigation/error state and
all 441 covariance entries, avoiding incompatible Eigen configuration across
translation units. Cases include all four update branches, nonzero bias/scale,
GNSS outage/recovery and 60 seconds from the upstream dataset. The equivalence
test is not independent navigation truth or evidence of H7 execution speed.
