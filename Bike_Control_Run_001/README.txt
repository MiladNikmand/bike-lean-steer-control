Bike Lean-Steer Control -- Run 001
Generated: 2026-09-13 17:37:01

BIKE PARAMETERS
  Forward speed:      1.00 m/s
  Rider+frame mass:   85.0 kg
  Wheelbase:          1.020 m
  Trail:              0.080 m
  Steer axis tilt:    18.00 deg
  Rear wheel radius:  0.300 m
  Open-loop regime:   UNSTABLE  (max Re = +3.5270)

INITIAL CONDITIONS
  phi_0:    5.00 deg
  delta_0:  -2.00 deg
  Duration: 30 s

CONSTRAINTS
  Roll limit (HARD STOP): +/- 90 deg  (bike fell)
  Steer saturation (clamp, NOT stop): +/- 60 deg
  Steer slew rate: unlimited (ideal actuator)
  Integration step dt: 0.0100 s

CONTROLLER RESULTS
  [uncontrolled]
    STATUS: FAILED at t=1.30s -- |phi|=91.4 deg > 90 deg (bike fell)
    ACTUATOR SATURATED: 19 times, first at t=0.93s

  [RateOnly]
    k=47.51 N*m*s/rad
    STATUS: OK
    ACTUATOR SATURATED: 14 times, first at t=0.53s
    Settle time (|phi|<1 deg): 3.80 s

  [PD]
    k1=250.0  k2=82.0
    STATUS: OK
    Settle time (|phi|<1 deg): 0.33 s

  [PID]
    k1=20.0  k2=188.0  k3=50.0
    STATUS: OK
    Settle time (|phi|<1 deg): 5.14 s

  [LQR]
    Q=diag[100 1 10 1]  R=0.10
    STATUS: OK
    Settle time (|phi|<1 deg): 0.81 s

  [FullState]
    poles: [-4 -5 -6 -7]
    STATUS: OK
    Settle time (|phi|<1 deg): 0.47 s

  [LQG]
    observes phi_dot only; estimates full state
    STATUS: FAILED at t=11.25s -- |phi|=90.7 deg > 90 deg (bike fell)
    ACTUATOR SATURATED: 123 times, first at t=0.59s

  [GainSched]
    zones: v<3->k=28.0 | 3-5.5->k=0.0 | v>5.5->k=0.0
    STATUS: OK
    ACTUATOR SATURATED: 923 times, first at t=0.53s

FILES
  figures/00_parameters.png       -- parameter summary
  figures/uncontrolled_response.png      -- response plot
  figures/uncontrolled_response.gif      -- animated response
  figures/RateOnly_response.png      -- response plot
  figures/RateOnly_response.gif      -- animated response
  figures/PD_response.png      -- response plot
  figures/PD_response.gif      -- animated response
  figures/PID_response.png      -- response plot
  figures/PID_response.gif      -- animated response
  figures/LQR_response.png      -- response plot
  figures/LQR_response.gif      -- animated response
  figures/FullState_response.png      -- response plot
  figures/FullState_response.gif      -- animated response
  figures/LQG_response.png      -- response plot
  figures/LQG_response.gif      -- animated response
  figures/GainSched_response.png      -- response plot
  figures/GainSched_response.gif      -- animated response
  figures/combined_comparison.png  -- all controllers
  results.mat                      -- raw simulation data
  run_summary.json                 -- machine-readable summary
