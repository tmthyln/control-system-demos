# Control Systems Demos

## Setup

### Python

1. Install Python
2. From the command line, install dependencies via
   ```shell
   pip install -r requirements.txt
   ```
   (You'll need to do this from the root directory of this repo.)
3. You can run each example from the command line, e.g.
   ```
   python pid/pendulum.py
   ```

### Julia

1. Install Julia using JuliaUp
2. Enter the Julia REPL and enter the following commands:
   ```julia
   import Pkg; Pkg.add(["Plots", "Pluto", "PlutoUI"])
   using Pluto
   Pluto.run()
   ```
3. A browser window will open for Pluto. You can select each notebook (which end in `.jl` to open and see the lesson/demo).

   (Upon loading a notebook initially, it may take a few minutes for the dependencies to install and the code to be compiled and run.)

## Answers

### Working PID constants
<details>
<summary>
Spoilers! Open for answers
</summary>

As discussed, these are not the only possible options.

Pendulum:
- P: 50
- I: 0
- D: -0.75

Car:
- P: 4
- I: 0.01
- D: -0.5

Cartpole:
- P: 10
- D: -1
</details>
