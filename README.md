# Hybrid Electric Vehicle Traction-Motor Speed Control

MATLAB and Simulink files for a Modern Control course project. The project designs and analyses feedback controllers for the speed-control loop of a hybrid electric vehicle traction motor, using root locus, frequency response, state-space feedback, and a block-diagram simulation in Simulink.

The plant is a simplified linearized model of a traction motor, driveline load, and speed sensor. A controller compares the commanded speed voltage with the speed-sensor voltage and adjusts the motor command. The project does not model the internal-combustion engine, battery, inverter, transmission, road load, or full vehicle dynamics.

A full walkthrough of the models, the method and the results is in [docs/REPORT.md](docs/REPORT.md).

## What is in the repository

| File | What it does |
|---|---|
| `controller_with_rootlocus.m` | Closes the loop around a PI-compensated traction-motor speed loop, plots its response to a 4 V speed-command step, then its response to a unit ramp together with the ramp error. |
| `controller_with_frequency.m` | Bode plot of the same PI-compensated traction-motor speed loop, followed by the 4 V step response of the closed loop. |
| `rootlocus.m` | Root locus of a third-order traction-motor speed loop with a 10 % overshoot line. It stops at `rlocfind` so you can click a gain on the locus, then plots the closed-loop step response for that gain. |
| `frequency_responce.m` | Bode plot and step response for the same third-order traction-motor speed loop with the gain fixed at 0.78. |
| `controller_with_linear_feedback.m` | State-space model with a state-feedback gain chosen by assigning closed-loop eigenvectors. Prints the closed-loop matrices and plots the step response. |
| `simulation.slx` | Simulink model with two nested PI loops around a simplified hybrid-vehicle traction-motor speed plant, driven by a 4 V speed-command step. |
| `docs/REPORT.md` | The report. |
| `docs/make_figures.py` | Python script that regenerates every figure and number in the report. |

The folder `simulation/` that came with the original project is just the unzipped contents of `simulation.slx`, so it does not need to be committed.

## Requirements

* MATLAB with the Control System Toolbox (`tf`, `feedback`, `bode`, `rlocus`, `rlocfind`, `sgrid`, `ss`)
* Simulink, for `simulation.slx`. The model uses the PID Controller and XY Graph blocks.
* Python 3 with `numpy`, `scipy` and `matplotlib`, only if you want to run `docs/make_figures.py`

## How to run

Most of the script names contain spaces, so MATLAB cannot call them by typing the name. Either open the file and press Run, or use `run`:

```matlab
run('controller_with_rootlocus.m')
run('controller_with_frequency.m')
run('rootlocus.m')
run('frequency_responce.m')
run('controller_with_linear_feedback.m')
```

Four of the scripts call `pause` after the first plot, so press any key in the command window to move on to the next plot. `rootlocus.m` waits for a mouse click on the root locus window after the plot appears. Click on the branch where you want the closed-loop poles and the script uses that gain.

To run the Simulink model, open `simulation.slx` and press Run. The stop time is 10 s. The two XY Graph blocks show the output against time and the output's derivative against time.

## Results at a glance

These numbers come from `docs/make_figures.py`, which reproduces the scripts in Python. They have not been read off MATLAB plots, so check them against your own runs.

| Design | Result |
|---|---|
| PI-compensated traction-motor loop, 4 V step | No visible overshoot, 2 % settling time about 0.08 s, final value 4 V |
| PI-compensated traction-motor loop, unit ramp | Steady-state error about 0.02 |
| PI-compensated traction-motor loop, Bode | Crossover near 49 rad/s, phase margin about 90° |
| Third-order traction-motor loop, gain 0.78 | Closed-loop poles at −0.403 ± 0.403j and −0.575, step overshoot about 20 %, phase margin about 66° |
| State feedback | Closed-loop poles at about −1.15 ± 1.22j |
| Simulink model, 4 V step | Output settles at about 5.56, overshoot about 12 %, settling time about 6.2 s |


## Author

Hossein Mosaffa, Modern Control Course Project at Amirkabir University of Technology, Faculty of Electrical Engineering.
