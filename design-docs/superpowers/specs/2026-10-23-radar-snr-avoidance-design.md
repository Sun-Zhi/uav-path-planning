# Radar SNR / RCS Avoidance — Design

**Date:** 2026-10-23  
**Status:** Approved (CN + US teaching profiles, SNR→Pd edge cost)

## Goal

Replace (or augment) soft AABB risk boxes with a **radar-equation detection model**: for each UAV sample point, compute SNR from each radar, map to detection probability \(P_d\), and fold \(\max P_d\) into A*/RRT* edge cost so paths avoid high-detectability volumes. Demo includes **two teaching profiles** inspired by open literature on US AN/SPY-6–class (S-band AESA) and CN JY-27–class (VHF early-warning), clearly labeled **not real classified parameters**.

## Physics (teaching approximation)

Monostatic radar equation (linear):

\[
\text{SNR} = \frac{P_t\,G^2\,\lambda^2\,\sigma}{(4\pi)^3\,R^4\,k\,T\,B\,F\,L}\cdot G_{\text{beam}}(\theta,\phi)
\]

Demo uses a compact equivalent:

\[
\text{SNR} = K\cdot \frac{\sigma}{R^4}\cdot G_{\text{beam}},\quad
K = \texttt{snr\_ref}\cdot R_{\text{ref}}^4 / \sigma_{\text{ref}}
\]

where `snr_ref` is the linear SNR at reference range \(R_{\text{ref}}\) for reference RCS \(\sigma_{\text{ref}}\) on boresight. Beam gain \(G_{\text{beam}}\in[0,1]\) is a raised-cosine / Gaussian falloff outside the half-power azimuth & elevation beamwidths (0 outside max instrumented range or below min elevation).

Detection probability (Swerling-1 style logistic gate):

\[
P_d = \frac{1}{1+\exp\!\big(-a(\ln\text{SNR}_{\text{dB}}-\text{snr\_thresh\_dB})\big)}
\]

with defaults \(a=0.35\), `snr_thresh_dB` from profile.

**Edge cost:** `length * (1 + w * Pd_edge)` where `Pd_edge` is the max \(P_d\) sampled along the segment (N=8 samples), `w` default 25. Buildings remain hard obstacles. Legacy AABB `risk_zones` still multiply if present (optional).

## Profiles (lab-scaled to 100 m map)

Real systems cite hundreds of km; **demo ranges are scaled** so both radars matter inside `bounds` 0–100 m. Labels in UI: `US SPY-6–class (teaching, scaled)` and `CN JY-27–class (teaching, scaled)`.

| id | Inspired by (open lit.) | Band (label) | Az/El HPBW | Scaled max range | Notes |
|----|-------------------------|--------------|------------|------------------|-------|
| `us_spy6_class` | AN/SPY-6 S-band AESA | S (~3 GHz label) | 12° / 10° | 55 m | Narrower beam, higher `snr_ref` |
| `cn_jy27_class` | JY-27 / JY-27A VHF | VHF (~300 MHz label) | 35° / 25° | 80 m | Wider beam; `rcs_scale` ≥1 so low-observable RCS less effective |

UAV default RCS \(\sigma=0.1\) m² (small UAS). JY-27 profile multiplies effective σ by `rcs_lo_factor` (e.g. 4) to reflect meter-wave reduced stealth benefit — **qualitative teaching only**.

## Architecture

1. `include/uav/radar.hpp` — `RadarSite`, `snr_linear`, `pd_from_snr`, `pd_at_point`, `max_pd_on_segment`.
2. `MapConfig` gains `std::vector<RadarSite> radars`, `double uav_rcs`, `double pd_cost_weight`.
3. `Environment::segment_risk_factor` / `edge_cost` incorporate radar \(P_d\) (and keep AABB soft zones).
4. JSON loader parses `"radars":[...]`.
5. `configs/sample_map.json` + `docs/sample_map.json` updated with two radars + start/goal still flyable.
6. WASM `plan` JSON adds `mean_pd`, `max_pd` along path; optional `pd_samples` omitted for size.
7. Demo UI draws translucent sector wedges / cones; legend names both profiles; stats show Pd.
8. `PAPERS.md` / README: disclaimer + open sources (Wikipedia SPY-6 S-band; radartutorial/ODIN JY-27 VHF ~240–390 MHz, instrumented ~390–500 km — **not used as numeric truth in demo**).

## Non-goals

Multipath, terrain masking, Doppler CFAR, jamming games, classified ERP/noise figures, unscaled hundreds-of-km geometry.

## Success

- Native `--demo` finds paths with lower mean Pd than a straight line through boresight.
- Pages demo shows both radar sectors; Plan reports path length + mean/max Pd.
- Disclaimers visible in README and page footer.
