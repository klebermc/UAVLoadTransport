# UAVLoadTransport

> **Note:** The simulation setup, controller models and learning-automata tuning experiments in this repository are by Kleber Cabral, built on third-party code credited below, including FALA training scripts by coauthor Sérgio R. Barros dos Santos, shared with his consent. The README documentation and the 2026-10-03 cleanup were done with AI assistance (Claude).

Code for the **2017 IEEE SysCon** paper "Design of Model Predictive Control via Learning
Automata for a Single UAV Load Transportation", by Kleber Macedo Cabral, Sérgio R. Barros
dos Santos, Sidney N. Givigi Jr., and Cairo L. Nascimento Jr.

**Paper:** [doi:10.1109/SYSCON.2017.7934800](https://doi.org/10.1109/SYSCON.2017.7934800) ·
**Slides:** [presentation/SysCon2017_slides.pdf](presentation/SysCon2017_slides.pdf)

## What it does

A quadrotor (Parallax ELEV-8 model) picks up and carries bricks of different mass and
balance in a V-REP scene, while its controllers run in Simulink. Learning automata tune
the controller parameters by running the simulation repeatedly and rewarding lower
tracking error. Two automata are used: a finite-action one (FALA) and a
continuous-action one (CARLA). The tuned controllers are compared against hand-tuned
PID on trajectory tracking with no load, a 250 g load, and an unbalanced 250 g load.

## Structure

- `src/` — MATLAB, Simulink and V-REP files, one folder per development stage. Folder
  and variable names are in Portuguese.
  - `Modelo_Simulink/` — Simulink-only quadrotor model with FALA training of the inner
    (attitude) and outer (position) PID loops.
  - `Modelo_Simulink_FALA/`, `Modelo_Simulink_CARLA/` — outer-loop training with each
    automaton, with the saved training workspaces.
  - `Modelo_Simulink_Gerar_Resultados/` — models and script that produce the RMS error
    and control-effort numbers for the hand-tuned and FALA-tuned controllers.
  - `VREP_Simulink_V1/`, `V2/`, `V4_LA_PID/` — Simulink controlling a quadrotor in
    V-REP through the remote API, from first link-up to FALA-tuned PID.
  - `VREP_Simulink_V5_Load_Transportation/` — the ELEV-8 load-transportation scene
    (`elev8_load_transportation.ttt`) and its controller model.
  - `VREP_Simulink_V6_CARLA/` — CARLA training of the inner and outer loops against
    the V-REP ELEV-8 model.
  - `CARLA_readme.txt` — the CARLA author's notice (see Third-party code).
- `figures/` — paper figure sources (`.odg`, `.png`, two `.fig`) and V-REP screenshots.
- `notes/` — learned gains, RMS comparison tables, and a V-REP + ROS + Simulink setup
  guide (in Portuguese).
- `presentation/SysCon2017_slides.pdf` — the conference talk slides.
- `videos/` — gitignored; the simulation videos shown in the talk.

## Not included

The Simulink model of the NMPC controller itself was not in the recovered project
folder. What is here is the simulation environment, the learning-automata tuning code,
and the PID baselines; the NMPC results survive only as figures
(`figures/NMPC_Responses_load250_unb.*`).

## Run

1. Copy the V-REP remote API MATLAB bindings into the stage folder (they are not
   redistributed here): `remApi.m` and `remoteApiProto.m` from
   `programming/remoteApiBindings/matlab/matlab/` and the `remoteApi` library from
   `programming/remoteApiBindings/lib/lib/` of your V-REP install.
2. Start V-REP and open the `.ttt` scene from the stage folder. `vrep_comm.m` connects
   to `127.0.0.1` on port 19997 and starts the simulation itself. To run V-REP on a
   second machine, as was done for the long training runs, change the IP in its
   `simxStart` call.
3. In MATLAB, `cd` to the same folder and open the `.mdl`/`.slx` model, or run the
   training script (`FALA_*.m`, `CARLA_*.m`), which opens the model itself.
4. `Modelo_Simulink*` folders need no V-REP: run the training script directly.

## Key dependencies

MATLAB/Simulink (some models also have an R2015a export), V-REP PRO EDU 3.3.2.

## Third-party code

- `mdl_quadrotor.m`, `quadrotor_dynamics.m`, `quadrotor_plot.m` — quadrotor model from
  Peter Corke's Robotics Toolbox (dynamics by Paul Pounds), with modified parameters.
- `betamax.m`, `calcbeta.m`, `density.m`, `expcarla.m`, `expect.m`, `init_cla.m`,
  `mep.m`, `ucp.m`, `show.m` — CARLA implementation by Mark Howell (Loughborough
  University), free to use and modify with acknowledgement.
- `FALA_LA_*_Controllers.m`, `FALA_OUTER_CONTROLLERS.m` and
  `Modelo_Simulink/controle.mdl` — learning-automata training code and base model by
  coauthor Sérgio R. Barros dos Santos, adapted here and shared with his consent.

The MIT license in this repo covers the remaining files only.

## Status

Paper code (SysCon 2017) — archival, not actively maintained. Recovered from a 2018
backup in October 2026; not re-run since.
