### A Pluto.jl notebook ###
# v0.19.26

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
end

# ╔═╡ a959c759-13bc-4365-8884-e8d3c294f2e4
# ╠═╡ show_logs = false
import Pkg; Pkg.add(["Plots", "PlutoUI"])

# ╔═╡ 980b772d-a1c3-441a-8887-32bf9d7ae0e7
using PlutoUI: Slider, TableOfContents

# ╔═╡ 7423eceb-9455-4d32-bdeb-cfec82bab90c
using Plots

# ╔═╡ aec67f2c-db9b-497a-a044-433fc110b698
md"""
# PID Control Theory Exploration

In this notebook, we'll build an intuition for PID controllers/loops, starting from why we need PID at all to the behavior of the controller in response to each of the parameters.

Julia knowledge is not required to go through the visualizations, but research shows early exposure to multiple languages is beneficial to cognitive development.
"""

# ╔═╡ 75ff43ff-0461-4418-b428-983a76191e26
md"""
Some housekeeping: we need to install and import some packages for the plotting, interactive widgets we'll use later on. Doing this here so you don't have to see these statements at the beginning of each section.
"""

# ╔═╡ b7caf330-8c20-419d-a782-6628e7e61b4d
TableOfContents()

# ╔═╡ 1557ba7a-be4d-4435-927b-6eed51fee78d
md"""
We will also set up some helper functions for the simulated example we used below:
"""

# ╔═╡ 1e8341fd-1f74-4964-8452-fa3716c26a96
function simulate(controller::Function; time_delta=0.05, num_iters=250_000)
	xs = (1:num_iters) .* time_delta
	cs = Vector{Float64}(undef, num_iters)
	ps = Vector{Float64}(undef, num_iters)

	pos::Float64 = 0
	vel::Float64 = 0

	for (index, x) in enumerate(xs)
		c = controller(pos, 100.0)
		cs[index] = c

		vel += c * time_delta
		pos += vel * time_delta + 0.5 * c * time_delta^2
		ps[index] = pos
	end

	return xs, cs, ps
end

# ╔═╡ 55cfddd9-b469-40f0-b2f9-35a3fd648074
plot_results(x_vals, c_vals, y_vals) = 
	plot(
		plot(x_vals, c_vals, ylabel="Control Variable\n(Force)"),
		plot(x_vals, y_vals, ylabel="Process Variable\n(Position)", xlabel="Time (sec)"),
		layout=(2, 1), 
		legend=false, 
	)

# ╔═╡ 0bb902e4-9083-4aca-8a4a-9d38fb47a8ea
md"""
## The Problem

So, before we talk about a solution (PID controllers), we need to first talk about the problem that it's supposed to solve. Otherwise, we might find ourselves in a bad case of trying to fit a problem to a solution we already have.

The problem is simple: we are using trying to control a variable

Concretely, for the robotics case: we have a **behavior** that we want a mechanism to do, a **sensor** that measures how close we are to achieving that behavior, and some **control** that we can adjust to make the mechanism get closer to (or farther away from) our goal.

### Method 1: Open-Loop Control

Sometimes we can ignore the sensor, and just turn on the mechanism, assuming that it does get to the setpoint. In FRC, this is often done with solenoids/piston: we switch the solenoid positions and assume that it's now the position we want it to be in. At most, we may wait a short period of time to ensure the piston's in place.

(I don't want to say this isn't ever the right choice, there are many situations where this is a perfectly fine control scheme.)

This obviously isn't great for situations where we need fine-grained control, so let's see our next option.

### Method 2: Bang-Bang Control

A funny name: bang when it starts, and bang when it's off.

Basically breaks down into 2 rules:
1. If we're not there, GO!
2. If we are there (or close enough), STOP!

Again, this is a simple control scheme that can work well in certain situations.
"""

# ╔═╡ ca8b523f-4de5-4315-8998-06af1afde3c9
md"""
Control Value: $(@bind bb_magnitude Slider(0:0.01:1, default=1, show_value=true)) 

(this is how much the mechanism is turned "on" when the controller says GO)

Tolerance: $(@bind bb_tolerance Slider(0:15, default=5, show_value=true))
"""

# ╔═╡ 4307e468-d66d-40cb-969c-090055b5cb22
bb_xs, bb_cs, bb_ys = simulate(num_iters=10_000) do measurement, setpoint
	if abs(setpoint - measurement) <= bb_tolerance
		return 0
	elseif measurement < setpoint
		return bb_magnitude
	else
		return -bb_magnitude
	end
end

# ╔═╡ ac8b3128-a101-4474-81eb-3061847927cc
plot_results(bb_xs, bb_cs, bb_ys)

# ╔═╡ 698f3d9d-29af-452c-a4f5-6bc0145f2267
md"""
**Reflection:**
- What are the issues with this control scheme?
- Can you think of some situations where this might work well?
"""

# ╔═╡ 10378073-8ceb-459f-81f1-964d9510804c
md"""
### Method 3: Proportional Control

Ok, so for most cases where we need fine control, this is probably not the best option. Looking at it, it's probably a bit coarse. Our control is basically a step function, with this harsh edge whenever we switch from "Not There" to "There."

A natural next step is: what if we slowly ramped up and down, instead of this jump?
"""

# ╔═╡ 5ea039f2-96b3-4246-908f-f1bc694d5fdf
md"""
Proportional Rate: $(@bind Kp_proportional Slider(0:0.00001:0.001, default=0, show_value=true))
"""

# ╔═╡ 4de71d80-cb6b-43ea-9519-52dd50540d6e
proportional_xs, proportional_cs, proportional_ys = simulate(num_iters=10_000) do measurement, setpoint
	error = setpoint - measurement
	return Kp_proportional * error
end

# ╔═╡ 9bb0d534-9166-4b95-8d1d-c47407f6e9d5
plot_results(proportional_xs, proportional_cs, proportional_ys)

# ╔═╡ 2e09ff3e-a6a9-43f7-bf20-848095220e5c
md"""
**Remarks**

- This appears remarkably similar to the bang-bang controller.
- What does the constant seem to control about the response?
- Note: this simulation assumes an ideal scenario, with no friction, outside disturbances, measurement error, or any of these real world issues. What would these curves look like if we added some of these back in?
"""

# ╔═╡ 1b5a96d5-5484-4400-a994-eb9d0a64517e
md"""
### Method 4: Integral Control

On its own, this control scheme doesn't seem very well motivated. The idea is that for every tick where we are not at the goal, we "ramp up our effort."

This one is difficult to simulate with our ideal example here: the motivation is that sometimes we don't give enough power for the system to actually get to the setpoint, so as long as we're not at the setpoint yet, we add a bit of power to our control variable.
"""

# ╔═╡ 818537a9-2fbe-4f3c-bc13-6c8d08e5edb1
md"""
Ki: $(@bind Ki_integral Slider(0:0.0000000001:0.00000001, default=0, show_value=true))

Tolerance: $(@bind integral_tolerance Slider(0:20, default=5, show_value=true))
"""

# ╔═╡ cbf8d5a7-e676-48f0-9bdc-838b4be57699
begin
	integral_accum = 0.0
	integral_xs, integral_cs, integral_ys = simulate(num_iters=100_000) do measurement, setpoint
		error = setpoint - measurement
		if abs(error) >= integral_tolerance
			integral_accum += error
		end
		
		return Ki_integral * integral_accum
	end
end

# ╔═╡ 015631e9-7f57-4de3-8a12-262aa4acb373
plot_results(integral_xs, integral_cs, integral_ys)

# ╔═╡ 41dd972f-d549-4d61-a0fb-dafe73e4a407
md"""
Hmm, ok, this doesn't seem to work very well. Do you have a guess why? Does this still hold if we combine control schemes to get a PD controller?
"""

# ╔═╡ 85287a94-c824-4df3-9557-c948560a0d22
md"""
### Method 5: Derivative Control

Right now, because we are operating in an ideal world, the proportional controller we have above never actually gets to the setpoint. That's upsetting, as that's the whole point of trying to do this.

Notice that we do hover "around" the setpoint: the oscillations go above and below 100 (which is our arbitrary goal). If only we could stop it from oscillating!

---

So clearly, proportional control isn't enough. What else can we do? Let's look closer at a single half-period of the curve (really just a sine curve in our ideal example):
"""

# ╔═╡ 488b4e71-0352-47a3-a125-3a1eee9b42ab
begin
	plot(0:0.1:π, 100 .+ sin.(0:0.1:π), legend=false)
	plot!(0:0.1:π/5, 100 .+ (0:0.1:π/5))
	plot!(π/4:π/2:3π/4, 101 .* ones(2))
end

# ╔═╡ a279190f-e9bc-400c-b645-e5f31002d7bd
md"""
I've drawn the derivatives here, which is a hint towards the next type of controller we might want to try. Recall that 100 is our setpoint (ignore the scale of the x-axis). 

1. At time 0, we've just overshot our target by a fairly large amount.
2. During the rising portion of the curve, our $P$ controller starts running in the opposite direction, trying to correct.
3. Eventually, our controller is able to get the controller to move the system back in the right direction. We're now super off our setpoint though!
4. During the falling portion of the curve, our $P$ controller provides a lot of control to the system (because we're so far away), so we accelerate a lot.
5. Oops, we're going too fast. We're going to overshoot again.

In a real-world system, we'll have pesky things like friction that means that we lose some energy to heat, so we won't do this forever. In this simulation, we ignore that, so we're kind of stuck in limbo.
"""

# ╔═╡ 5a4533d8-0d5b-4fb2-9462-3a181575573d
md"""
**An Answer**

Notice the tangent lines that are drawn in the image above.

At (0, 100),
- we have a lot of momentum, but
- the controller provides no power to the system! Because we can't just stop the system immediately, we should actually provide some power opposite to the direction of motion. (This is why sometimes you can think about this as anticipating the "future error.")

At the peak,
- we have no momentum, but
- now the controller is providing too much power to the system! We'd like this to be smaller (although ideally, we don't ever oscillate this high).

Hmm, it seems like
- we want to provide a strong-ish (negative) control when we have too much momentum -- hey, the derivative seems to be large there!
- we don't want to provide much whenever we *don't* have that excess momentum - hey again, the derivative seems to be close to 0 there!
"""

# ╔═╡ e547f5b3-fb8e-45c2-b9bf-d8fe8d3198bd
md"""
Kp: $(@bind Kp_pd Slider(0:0.00001:0.001, default=0, show_value=true))

Kd: $(@bind Kd_pd Slider(-0.1:0.001:0, default=0, show_value=true))
"""

# ╔═╡ ff468b6a-69ed-491f-bac5-ef68020100a6
begin
	previous_measurement::Union{Float64, Nothing} = nothing
	pd_xs, pd_cs, pd_ys = simulate(num_iters=100_000) do measurement, setpoint
		error = setpoint - measurement
		time_delta = 0.05

		if previous_measurement === nothing
			diff = 0
		else
			diff = (measurement - previous_measurement) / time_delta
		end
		previous_measurement = measurement
		
		return Kp_pd * error + Kd_pd * diff
	end
end

# ╔═╡ da9335d0-53ea-42b0-b905-84d5de9767e1
plot_results(pd_xs, pd_cs, pd_ys)

# ╔═╡ a8e77573-1fd9-4b41-a424-4d6f960f2176
md"""
**Reflect**:
- What problem is derivative control trying to fix?

**Food for Thought**:
- We need derivative control here because we said we can't stop the system immediately. Are there situations where this assumption is not true?
"""

# ╔═╡ 6d4e8495-2601-491e-8b0e-36592f350802
md"""
### Method X: Putting it all together: the PID Controller

Now we're ready to play around with the full PID controller. This is just the P, I, and D equations added together.

To build on the idea above about "future error":

- **P**: accounts for the "current or present error"
- **I**: accounts for "past error"
- **D**: accounts for "future error"

---

Here's a struct and a couple functions for the implementation (you can open these blocks up if you're interested):
"""

# ╔═╡ 373dc39b-ff02-4b99-9e78-260c616f6dd8
mutable struct PIDController{M <: Number}
	p::M
	i::M
	d::M
	last_measurement::Union{M, Nothing}
	accumulator::M
	time_delta::M

	function PIDController{M}(p=0, i=0, d=0, time_delta=0.05) where {M}
		new(p, i, d, nothing, zero(M), time_delta)
	end
end

# ╔═╡ ed3dfcd6-df8d-4d1b-9621-cbd71bb8a681
function reset!(pid::PIDController{M}) where {M}
	pid.accumulator = zero(M)
end

# ╔═╡ 027cb0bb-064a-41a8-9746-e73cac8830a3
function step!(pid::PIDController{M}, measurement::M, setpoint::M) where {M}
	error = setpoint - measurement
	pid.accumulator += error

	if pid.last_measurement === nothing
		diff = 0
	else
		diff = (measurement - pid.last_measurement) / pid.time_delta
	end

	pid.last_measurement = measurement

	return pid.p * error + pid.i * pid.accumulator + pid.d * diff
end

# ╔═╡ 1c13f9d9-85e8-4f03-9a82-ed253a18c7d3
md"""
Here's the PID controller:
"""

# ╔═╡ e88eaa23-4a2a-4e8e-9b10-845e2f2aa299
md"""
Kp = $(@bind p Slider(0:0.00001:0.5, default=0.0, show_value=true))

Ki = $(@bind i Slider(-0.01:0.0001:0.01, default=0.0, show_value=true))

Kd = $(@bind d Slider(-01:0.0001:0.001, default=0.0, show_value=true))
"""

# ╔═╡ 48d8fabe-6c1e-4a71-86d0-e934d6b1691a
begin
	pid = PIDController{Float64}(p, i / 100, d)
	xs, cs, ys = simulate(num_iters=50000) do measurement, setpoint
		step!(pid, measurement, setpoint)
	end
end

# ╔═╡ d248cd28-5a1d-48b8-b58b-e2e818ab1dd6
plot_results(xs, cs, ys)

# ╔═╡ b36b460e-c76e-4036-a5b3-de1e2a0bcf9e
md"""
## Tuning Methods

Now, for the practical part. Our goal is to actually use these PID controllers, so we need to be able to figure out useful values to use for P, I, and D. We could guess and check, but in the real world we don't have the benefit of being able to simulate all of our control needs all the time.

Have you figured out any tuning methods?

### Manual Tuning

While there are many ways to manually tune a PID controller, here's a procedure that generally works well:

1. Start with $P = 0$, $I = 0$, and $D = 0$.
2. Slowly increase $P$ until you get a bit of oscillation around your setpoint.
3. Now, you can slowly decrease $D$ until you achieve the damping that you want.
4. Ignore $I$. If you *really* want, you can add a *little, little* bit of $I$ if you're really not getting to your setpoint.

Try this in the simulation above to see what happens to your system's response as you do each step.


### Formulaic Methods

Over the years, there are also heuristic approaches that can give some rough ideas of what to use for your PID constants, based on more or less arbitrary design goals. One of the most famous is the [Ziegler-Nichols method](https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method).

If you try this method above, does it work? (You may need to reverse the sign of the $K_d$ constant.) Any thoughts?
"""

# ╔═╡ 15a3c341-d17d-4506-9267-f01bddde260a
md"""
**Food for Thought**

Why isn't there just some formula where we plug in our system's parameters and it outputs the ideal $K_p$, $K_i$, and $K_d$ constants?

- Is the goalpost static? Does the ideal response curve look different for different situations, mechanisms, or preferences? 
  - Sometimes, some overshoot is okay if we get to our setpoint faster. Are there situations where overshoot is problematic, or even unacceptable?
- Maybe we could come up with such a formula for our contrived and simulated example above. What factors do we not account for from the real world?
"""

# ╔═╡ 76d47acf-b6dd-4240-b604-e49affd4f074
md"""
## Now It's Your Turn! (cheesy educational slang)

We're going to move from unrealistic simulations to slightly more realistic simulations to get some practice tuning PID controllers.

In the same repository where this notebook is, there's a set of files containing some experiments (in Python) where you can tune a PID controller (or implement your own separate controller!) to try to solve each of the Reinforcement Learning environments.
"""

# ╔═╡ b0500a53-1df9-493d-a64c-222e01d9ad8e
md"""
### Other Ideas

- Do you have any other ideas for how to build a controller?
- Try to think about how to adapt this controller if, say, you
  - Had 2 or more equivalent sensors?
  - Need to control multiple outputs together? Does it make a difference if the outputs are correlated vs. independent?
"""

# ╔═╡ Cell order:
# ╟─aec67f2c-db9b-497a-a044-433fc110b698
# ╟─75ff43ff-0461-4418-b428-983a76191e26
# ╠═a959c759-13bc-4365-8884-e8d3c294f2e4
# ╠═980b772d-a1c3-441a-8887-32bf9d7ae0e7
# ╠═b7caf330-8c20-419d-a782-6628e7e61b4d
# ╠═7423eceb-9455-4d32-bdeb-cfec82bab90c
# ╟─1557ba7a-be4d-4435-927b-6eed51fee78d
# ╠═1e8341fd-1f74-4964-8452-fa3716c26a96
# ╟─55cfddd9-b469-40f0-b2f9-35a3fd648074
# ╟─0bb902e4-9083-4aca-8a4a-9d38fb47a8ea
# ╟─ca8b523f-4de5-4315-8998-06af1afde3c9
# ╟─4307e468-d66d-40cb-969c-090055b5cb22
# ╟─ac8b3128-a101-4474-81eb-3061847927cc
# ╟─698f3d9d-29af-452c-a4f5-6bc0145f2267
# ╟─10378073-8ceb-459f-81f1-964d9510804c
# ╟─5ea039f2-96b3-4246-908f-f1bc694d5fdf
# ╠═4de71d80-cb6b-43ea-9519-52dd50540d6e
# ╟─9bb0d534-9166-4b95-8d1d-c47407f6e9d5
# ╟─2e09ff3e-a6a9-43f7-bf20-848095220e5c
# ╟─1b5a96d5-5484-4400-a994-eb9d0a64517e
# ╟─818537a9-2fbe-4f3c-bc13-6c8d08e5edb1
# ╠═cbf8d5a7-e676-48f0-9bdc-838b4be57699
# ╟─015631e9-7f57-4de3-8a12-262aa4acb373
# ╟─41dd972f-d549-4d61-a0fb-dafe73e4a407
# ╟─85287a94-c824-4df3-9557-c948560a0d22
# ╟─488b4e71-0352-47a3-a125-3a1eee9b42ab
# ╟─a279190f-e9bc-400c-b645-e5f31002d7bd
# ╟─5a4533d8-0d5b-4fb2-9462-3a181575573d
# ╟─e547f5b3-fb8e-45c2-b9bf-d8fe8d3198bd
# ╠═ff468b6a-69ed-491f-bac5-ef68020100a6
# ╟─da9335d0-53ea-42b0-b905-84d5de9767e1
# ╟─a8e77573-1fd9-4b41-a424-4d6f960f2176
# ╟─6d4e8495-2601-491e-8b0e-36592f350802
# ╠═373dc39b-ff02-4b99-9e78-260c616f6dd8
# ╟─ed3dfcd6-df8d-4d1b-9621-cbd71bb8a681
# ╠═027cb0bb-064a-41a8-9746-e73cac8830a3
# ╟─1c13f9d9-85e8-4f03-9a82-ed253a18c7d3
# ╠═e88eaa23-4a2a-4e8e-9b10-845e2f2aa299
# ╠═48d8fabe-6c1e-4a71-86d0-e934d6b1691a
# ╟─d248cd28-5a1d-48b8-b58b-e2e818ab1dd6
# ╟─b36b460e-c76e-4036-a5b3-de1e2a0bcf9e
# ╟─15a3c341-d17d-4506-9267-f01bddde260a
# ╟─76d47acf-b6dd-4240-b604-e49affd4f074
# ╟─b0500a53-1df9-493d-a64c-222e01d9ad8e
