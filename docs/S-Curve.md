S-Curve acceleration
====================

This document provides an overview of a special mode of acceleration implemented
in Klipper called S-Curve acceleration. This acceleration scheme replaces the
traditional "[trapezoid generator](Kinematics.md#trapezoid-generator)" by
modeling higher-order derivatives, like jerk, snap and so forth, depending on
the S-Curve order, using Bezier polynomials.

**Warning**: S-Curve support is experimental. You should consider using S-Curve
acceleration only if you already have some problems with the prints, otherwise
it is not advised to switch over from the default trapezoid generator.
Examples of problems S-Curve acceleration may help with:

* Ghosting and ringing in the prints
* Extruder skipping steps or rattling when using
  [Pressure Advance](Pressure_Advance.md):
    * bowden extruder with very high pressure advance value
    * direct extruder with high gear ratio (e.g. remote direct drive extruder)

The improvements do not come for free: your prints will likely slow down.
Depending on the model and print parameters, slow down can be from negligible
to 20-30% and more. On the other hand, the same level of improvements usually
cannot be obtained just by reducing the acceleration.

Note that ghosting usually has mechanical origins: insufficiently rigid printer
frame, non-tight or too springy belts, alignment issues of mechanical parts,
heavy moving mass, etc. Those should be checked and fixed first.


Switch to S-Curve acceleration branch
===========================

To try experimental S-Curve acceleration mode in your existing Klipper
installation, SSH to your Raspberry Pi and run the following commands:
```
$ cd klipper
$ sudo service klipper stop
```

Configure the new Git remote:
```
$ git remote add s-curve-exp https://github.com/dmbutyugin/klipper.git
$ git remote -v
```
The output should list the new remote among other things:
```
s-curve-exp	https://github.com/dmbutyugin/klipper.git (fetch)
s-curve-exp	https://github.com/dmbutyugin/klipper.git (push)
```

Now check the current branch, it will be needed to roll back after you are
finished with the experiments:
```
$ git branch
```
will most likely list
```
* master
```

Check out the new branch:
```
$ git fetch s-curve-exp
$ git checkout s-curve-exp/scurve-smoothing
```

Start Klipper:
```
$ sudo service klipper start
```

If you want to switch back to the main Klipper branch, SSH to your Raspberry
Pi and run the following commands:
```
$ cd klipper
$ sudo service klipper stop
$ git checkout master
$ sudo service klipper start
```

Basic tuning
===========================

Basic tuning requires measuring the ringing frequencies of the printer and
adding a few parameters to `printer.cfg` file.


Slice the ringing test model, which can be found in
[docs/prints/ringing_tower.stl](prints/ringing_tower.stl), in the slicer:

 * Suggested layer height is 0.2 or 0.25 mm.
 * Infill and top layers can be set to 0.
 * Use 1-2 perimeters, or even better the smooth vase mode with 1-2 mm base.
 * Use sufficiently high speed, around 80-100 mm/sec, for *external* perimeters.
 * Make sure that the minimum layer time is *at most* 3 seconds.

## Ringing frequency

First, measure the **ringing frequency**. Note that these measurements can also
be done on the mainline Klipper branch before switching to the S-Curve branch.

1. Increase `max_accel` and `max_accel_to_decel` parameters in your
   `printer.cfg` to 7000.
2. Restart the firmware: `RESTART`.
3. Disable Pressure Advance: `SET_PRESSURE_ADVANCE ADVANCE=0`.
4. If you have already switched to the S-Curve branch and updated the config,
   execute `SET_SCURVE ACCEL_ORDER=2` and
   `SET_SMOOTH_AXIS SMOOTH_X=0 SMOOTH_Y=0` commands. If you get
   "Unknown command" errors for any of these commands, you can safely ignore
   them at this point and continue with the measurements.
5. Execute the command
   `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.
   Basically, we try to make ringing more pronounced by setting different large
   values for acceleration.
6. Print the test model sliced with the suggested parameters.
7. You can stop the print earlier if the ringing is clearly visible and you see
   that acceleration gets too high for your printer.
8. Measure the distance *D* (in mm) between *N* oscillations for X axis near
   the notches, preferably skipping the first oscillation or two. Pay attention
   to the notches X axis corresponds to - the test model has large 'X' and 'Y'
   marks on the back side for convenience. Note that 'X' mark is on Y axis and
   vice versa, it is not a mistake - ringing of X axis shows *along* Y axis.
   To measure the distance between oscillations more easily, mark the
   oscillations first, then measure the distance between the marks with a ruler
   or calipers:

    |![Mark ringing](img/ringing-mark.jpg)|![Measure ringing](img/ringing-measure.jpg)|
    |:--:|:--:|

9. Compute the ringing frequency = *V* &middot; *N* / *D* (Hz) where *V* is the
   velocity for outer perimeters (mm/sec). For the example above, we marked 6
   oscillations, and the test was printed at 100 mm/sec velocity, so the
   frequency is 100 * 6 / 12.14 ~= 49.4 Hz.
10. Do (8) - (9) for Y axis as well.

Note that ringing on the test print should follow the pattern of the curved
notches, as in the picture above. If it doesn't, then this defect is not really
a ringing and has a different origin - either mechanical, or an extruder issue.
It should be fixed first before enabling and tuning S-Curve mode.

For Cartesian printers, you will obtain 2 frequencies (f<sub>X</sub> and
f<sub>Y</sub>), which may be different, especially on bed-slinger printers.
CoreXY printers could have 4 frequencies. It is possible, but not mandatory, to
repeat the process (2)-(10) rotating the model 45 degrees around Z axis such
that the angle between the X and Y axes and the sides of the model is
45 degrees; then measure 2 extra frequencies. Delta printers should have
3 frequencies close to each other; just measure 2 frequencies as for Cartesian
printers.

Ringing frequency can depend on the position of the model within the buildplate
and Z height; you can check if you see the differences in frequencies at
different positions along the sides of the test model and at different heights.
On the other hand, the measurements do not have to be very precise, and some
variations in the ringing frequencies are tolerable. You can calculate
the average ringing frequencies over X and Y axes.

Note that the ringing frequencies can change if the changes are made to the
printer that affect the moving mass or change the stiffness of the system,
for example:

  * Some tools are installed, removed or replaced on the toolhead that change
    its mass, e.g. a new (heavier or lighter) stepper motor for direct extruder
    or a new hotend is installed, heavy fan with a duct is added, etc.
  * Belts are tightened.
  * Some addons to increase frame rigidity are installed.
  * Different bed is installed on a bed-slinger printer, or glass added, etc.

If such changes are made, it is a good idea to at least measure the ringing
frequencies to see if they have changed. If they changed significantly, the
full S-Curve tuning process should be done (except tuning Pressure Advance).

## Basic configuration

After the ringing frequencies for X and Y axes are measured, you can add the
following section to your `printer.cfg`:
```
[smooth_axis]
smooth_x: ... # 2 / (3 * f_X)
smooth_y: ... # 2 / (3 * f_Y)
accel_comp_x: ... # 1 / (2 * π * f_X)^2
accel_comp_y: ... # 1 / (2 * π * f_Y)^2
```

For the example above, we get:

  * smooth_x/y = 2 / (3 * 49.4) ~= 0.0135 and
  * accel_comp_x/y = 1 / (2 * π * 49.4)^2 ~= 1 / 310.4^2 ~= 0.00001.

You can print the ringing test model now following the steps (1)-(6) from above.
If you see that the ringing is gone, you can stop the tuning process at this
point and not configure S-Curves. Just restore the original values for
`max_accel` and `max_accel_to_decel` parameters in your `printer.cfg`.

If you still see some ringing up to and a bit above the accelerations you
normally print at (e.g. if you normally print at 3000 mm/sec^2, check
accelerations up to ~4500 mm/sec^2, but if you see the ringing only at
7000 mm/sec^2, it is most likely not a problem), add the following section:
```
[scurve]
acceleration_order: 4
min_jerk_limit_time: ...  # 1.0 / max(f_X, f_Y)
```
For 49.4 Hz frequency we get min_jerk_limit_time ~= 0.02.

Now you can restore the original `max_accel` and `max_accel_to_decel` values in
your `printer.cfg` (simply remove `max_accel_to_decel` if you did not use it
previously) and restart Klipper. The basic tuning is complete.


Fine tuning
===========================

This section describes additional tuning that can be done to improve S-Curves
performance. As a result, you may be able to speed up your prints, getting close
to the performance of the mainline Klipper branch, and, if you are lucky, even
exceeding it, without inducing the ringing in the prints. Or, if you still see
some ringing in your prints after the basic tuning, you may be able to reduce
it further following the steps outlined below.

Tuning procedure below assumes that you have completed the basic tuning already,
and have configured the S-Curve mode as described there. Note that it is also
possible to run fine-tuning from this section, except "Max jerk tuning", if only
`[smooth_axis]` was enabled, but not `[scurve]`. You can follow the instructions
below as-is, simply ignoring "Unknown command" errors for `SET_SCURVE` command.

## Best tuning conditions

First, you will need to perform the following test to find the best
tuning conditions:

1. Slice the test ringing_test.stl [model](prints/ringing_tower.stl) as
   suggested in the [basic tuning](#basic-tuning) section and upload it to
   the printer.
2. Increase `max_accel` and `max_accel_to_decel` parameters in your
   `printer.cfg` to 7000.
3. Restart the firmware: `RESTART`.
4. Disable Pressure Advance: `SET_PRESSURE_ADVANCE ADVANCE=0`.
5. Disable acceleration compensation:
   `SET_SMOOTH_AXIS ACCEL_COMP_X=0 ACCEL_COMP_Y=0`.
6. Set very high max jerk parameter, e.g. `SET_SCURVE JERK=2000000`.
7. Execute the command
   `TUNING_TOWER COMMAND=SET_VELOCITY_LIMIT PARAMETER=ACCEL START=1250 FACTOR=100 BAND=5`.
8. Print the test model sliced with the suggested parameters.
9. You can stop the print earlier if the ringing is clearly visible and you see
   that acceleration gets too high for your printer.

Take a note which band shows ringing the best and count its number from the
bottom starting at 1. Calculate the corresponding acceleration as
1000 + 500 * #band-number. For example, if you see that the fifth band
from the bottom shows ringing pretty well, it corresponds to the acceleration
1000 + 500 * 5 = 3500 mm/sec^2. We will use this acceleration value
in the next steps.

## Acceleration compensation tuning

The feature is controlled by `accel_comp_x` and `accel_comp_y` parameters in
`[smooth_axis]` section. Acceleration compensation further reduces the ringing
without impacting the speed of the prints. The theory suggests that the best
value of acceleration compensation for an axis is equal to `1 / (2 π f)^2` with
f being the resonance (ringing) frequency for that axis. For example, for 50 Hz
ringing frequency accel_comp_x/y ~= 0.00001 (sec^2). However, it is possible
that the actual ideal values for these parameters slightly deviate from the
suggested ones. So it might be a good idea to tune them to further reduce the
ringing.

Assuming that you have sliced the ringing model with suggested parameters and
increased `max_accel` and `max_accel_to_decel` parameters in the `printer.cfg`
to 7000 already, complete the following steps for each of the axes X and Y:

1. Make sure Pressure Advance is disabled: `SET_PRESSURE_ADVANCE ADVANCE=0`.
2. Set the acceleration value obtained at the previous step that shows ringing
   the best: `SET_VELOCITY_LIMIT ACCEL=...` (insert the value you have
   obtained previously).
3. Set the same max jerk parameter you used to measure the acceleration, e.g.:
   `SET_SCURVE JERK=2000000`.
4. Calculate the necessary parameters for the `TUNING_TOWER` command to tune
   `accel_comp_x` parameter as follows: start = accel_comp_x * 17 / 66 and
   factor = accel_comp_x / 33, where `accel_comp_x` here is the current value
   in `printer.cfg` (e.g. obtained during the basic tuning, which is equal to
   1 / (2 * π * f_X)^2).
5. Execute the command
   `TUNING_TOWER COMMAND=SET_SMOOTH_AXIS PARAMETER=ACCEL_COMP_X START=start FACTOR=factor BAND=5`
   using `start` and `factor` values calculated at step (4).
6. Print the test model.
7. Reset the original acceleration compensation value:
   `SET_SMOOTH_AXIS ACCEL_COMP_X=...`.
7. Find the band which shows ringing the least and count its number from the
   bottom starting at 1.
8. Calculate the new accel_comp_x value via old
   accel_comp_x * (6 + 5 * #band-number) / 33.

Repeat these steps for the Y axis in the same manner, replacing references to X
axis with the axis Y (e.g. replace `accel_comp_x` with `accel_comp_y` in the
formulae and in the `TUNING_TOWER` command).

As an example, let's assume you have had measured the ringing frequency for one
of the axis equal to 45 Hz. Then the original accel_comp_? parameter should have
been ~= 0.0000125 (sec^2). This gives start = 0.0000125 * 17 / 66 = 0.00000322
and factor = 0.0000125 / 33 = 0.000000379 values for `TUNING_TOWER` command.
Now let's assume that after printing the test model, the fourth band from the
bottom gives the least ringing. This gives the updated accel_comp_? value equal
to 0.0000125 * (6 + 5 * 4) / 33 ~= 0.00000985.

Note that `start` and `factor` values suggested at step (4) are good all-round
values. If you see no improvements over the test print, it can mean that
there are other issues in the printer that make acceleration compensation
unhelpful (you can either revert the corresponding value of `accel_comp_?`
to the default value from basic tuning or just set it to 0 in this case), or you
may need to test a wider range of values (in the latter case refer to the
documentation on `TUNING_TOWER` command and try different `start` and `factor`
that cover larger or smaller values).

After both new `accel_comp_x` and `accel_comp_y` parameters have been
calculated, you can update `[smooth_axis]` section in `printer.cfg` with the
new `accel_comp_x` and `accel_comp_y` values.

Also, if you see some improvements around the first or the last band for an
axis, you can repeat the tuning process in this section iteratively, using
accel_comp_x/accel_comp_y obtained on the previous iteration as initial values
for the next iteration (instead of the default values from basic tuning).

## Pressure Advance

If you use Pressure Advance, it may need to be re-tuned. Follow the
[instructions](Pressure_Advance.md#tuning-pressure-advance) to find the
new value, if it differs from the previous one. Make sure to restore the
original values of `max_accel` and `max_accel_to_decel` parameters in the
`printer.cfg` and restart Klipper before tuning Pressure Advance.

If later during printing you notice that extruder rattles or skip steps, it
means that the pressure advance value is too high for the corresponding
acceleration. You can either reduce `max_accel` setting, or tune `max_jerk`
value explicitly. Follow the suggestions in [max jerk tuning](#max-jerk-tuning)
section below.


## Max jerk tuning

There is additional configuration parameter - `max_jerk` - that you can try to
tune to increase the speed of the your prints. The default value for it is
equal to 0.6 * max_accel * min_jerk_limit_time (when no value is set in the
config file).

Keep in mind that there is no well-defined upper limit on the maximum jerk
value. Still, the practical range of values to choose max_jerk from is from
around 50'000 to 6 * max_accel / min_jerk_limit_time (mm/sec^3), and potentially
lower when using Pressure Advance, depending on the printer setup and hardware.

Assuming that you have sliced the ringing model with suggested parameters
already, complete the following steps to tune max_jerk parameter:

1. Increase `max_accel` and `max_accel_to_decel` parameters in your
   `printer.cfg` to 7000 and restart Klipper.
2. Set the acceleration value obtained [previously](#best-tuning-conditions)
   that shows ringing the best: `SET_VELOCITY_LIMIT ACCEL=...`.
3. Calculate the necessary parameters for the `TUNING_TOWER` command to tune
   `max_jerk` parameter as follows:
   start = 0.15 * original_max_accel / min_jerk_limit_time and
   factor = 0.06 * original_max_accel / min_jerk_limit_time,
   where original_max_accel is the max_accel parameter value you normally use.
4. Execute the command
   `TUNING_TOWER COMMAND=SET_SCURVE PARAMETER=JERK START=start FACTOR=factor BAND=5`
   using `start` and `factor` values calculated at step (3).
5. Print the test model.
6. Find the last band from the bottom which still shows no ringing and count its
   number from the bottom starting at 1. You may need to stop earlier than that
   if you notice that extruder cannot keep up with Pressure Advance anymore -
   use the band number where extruder still works normally instead.
7. Calculate the new max_jerk value as
   0.3 * original_max_accel / min_jerk_limit_time * #band-number.

For example, if you normally use max_accel = 5000 mm/sec^2, and have previously
set min_jerk_limit_time = 0.0182 (corresponds to 55 Hz ringing), the parameters
for TUNING_TOWER command are start = 0.15 * 5000 / 0.0182 ~= 41200 and factor =
0.06 * 5000 / 0.0182 ~= 16500. If the last band with satisfactory results is
7-th, we get max_jerk = 0.3 * 5000 / 0.0182 * 7 ~= 577000 (mm/sec^3).

Now restore the original values of `max_accel` and `max_accel_to_decel`
parameters and put the calculated value into `[scurve]` section in
`printer.cfg`:
```
[scurve]
max_jerk: ... # 0.3 * max_accel / min_jerk_limit_time * #band-number
```
After restarting Klipper, the tuning process is complete.

S-Curve acceleration overview
=============================

By default, Klipper uses "trapezoid generator" for each move - each move has
a start speed, it accelerates to a cruising speed at constant acceleration,
it cruises at a constant speed, and then decelerates to the end speed using
constant acceleration.

![trapezoid](img/trapezoid.svg.png)

The basic idea behind S-Curve acceleration is to replace the constant
acceleration and deceleration with polynomials of higher order. For some
choices of the polynomials the toolhead will travel the same path and reach
the same speed as if it was accelerating constantly, but higher order
derivatives (e.g. acceleration, jerk) can be 0 in the beginning and at the end
of acceleration depending on the polynomial order, ensuring better smoothness
of the motion.

Klipper currently supports acceleration_order = 2, 4 and 6.
acceleration_order = 2 is the default constant acceleration mode.
Charts below show the distance covered, velocity and acceleration for different
acceleration orders.

|![Distance](img/s-curve-x.svg.png)|
|:--:|
| *Distance* |
|![Velocity](img/s-curve-v.svg.png)|
| *Velocity* |
|![Acceleration](img/s-curve-a.svg.png)|
| *Acceleration* |

Notice that the velocity and acceleration are smoother with higher acceleration
orders. When acceleration is not constant, the 'lack' of acceleration in
the beginning must be compensated with higher max acceleration. Klipper still
plans all moves considering the 'effective' acceleration, which can be seen as
an average acceleration, but then each move is executed using the polynomial
of the chosen degree. Though instantaneous acceleration exceeds the configured
maximum toolhead acceleration, because the movement is smoother overall,
the reduction of the maximum permitted acceleration is usually not necessary.

Besides making the movements smoother, S-Curve acceleration *may* improve the
quality of the prints. One of the theories behind it is that each printer has
limited non-infinite rigidity of the frame, belts, etc. When the force is
applied or relieved instantly in the beginning and at the end of acceleration
or deceleration, the system can act as a spring and start oscillating, which
can be observed in the form of ringing in the prints. S-Curve acceleration
'spreads' increase and decrease of the force during a longer time, potentially
reducing the oscillations.

This has an important consequence: if the short moves are executed with the
same acceleration, the full force must be applied over the shorter period of
time, effectively nullifying the positive effect of S-Curve acceleration on
short moves. That's why Klipper also limits the maximum kinematic jerk
*J* = *da* / *dt* of each acceleration and deceleration in S-Curve acceleration
mode.

For the chosen polynomials, the maximum kinematic jerk *J* is

*J* = 6 *a* / *T*

for acceleration_order = 4, where *a* is effective acceleration and *T* is
acceleration time. For acceleration_order = 6

*J* = 10 *a* / (*T* &#8730;3) &asymp; 5.774 *a* / *T*.

In the end, we use 6 *a* / *T* < max_jerk condition to limit the jerk. This
leads to a cubic equation

(*v*<sup>2</sup> - *v*<sub>0</sub><sup>2</sup>) &times;
  (*v* + *v*<sub>0</sub>) / 2 = *L*<sup>2</sup> *J* / 3,

which can be solved using Cardano's formula to determine the velocity *v*
after acceleration with the maximum jerk *J* over the segment of length *L*.
The final velocity is chosen as a minimum out of that value and
*v*<sub>0</sub> + (2 *a* *L*)<sup>1/2</sup>.

Another reason to limit the jerk is that it directly translates into extruder
acceleration *a*<sub>e</sub> if Pressure Advance is enabled:
*a<sub>e</sub>* = *r P J*,
where *P* is the pressure advance parameter,

*r* = (4 *w* *h*) / (&pi; *D*<sup>2</sup>).

is the extrusion ratio, *D* is the filament diameter, *w* is the extrusion
width, *h* is the layer height. As an example, with *P* = 0.5, *w* = 0.4 mm,
*h* = 0.2 mm, *D* = 1.75 mm, and *J* = 100'000 the extruder acceleration is
1663 mm/sec^2 due to jerk.

Extruder kinematics looks as follows with different acceleration orders:

|![Velocity](img/s-curve-ev.svg.png)|
|:--:|
| *Extruder Velocity* |
|![Acceleration](img/s-curve-ea.svg.png)|
| *Extruder Acceleration* |

Notice the velocity jump with acceleration_order = 2 (the 'infinite'
acceleration spikes at the beginning and the end of acceleration with
acceleration_order = 2 are not shown). With acceleration_order > 2
the velocity is continuous, and for acceleration_order = 6 it is even
smooth.  Thus, acceleration_order > 2 can improve the performance of
the extruder if pressure advance is enabled.

## Acceleration compensation

As the toolhead accelerates or decelerates, it can deviate from the commanded
trajectory due to flex in belts and small frame deformations. When accelerating,
it usually 'lags' from the commanded position, and when decelerating -
overshoots it a bit (illustration with ringing at 40 Hz):

![Toolhead cornering](img/toolhead-cornering.png)

This effect can impact the quality of the corners even at very small corner
velocities and increase the ringing. S-Curve acceleration has an experimental
mode called acceleration compensation which can help to mitigate this issue to
some degree.


S-Curve acceleration notes
==========
 * S-Curve reduces acceleration of short moves, effectively slowing them down
    * this can be disabled by setting very high `max_jerk` value in the config,
      e.g. 1000000, but this is not advised, unless acceleration compensation
      is enabled
    * jerk limit is not applied to extrude-only moves, they are always executed
      with the full `max_extrude_only_accel` acceleration.
 * When the move must both accelerate and decelerate, and acceleration for the
   move is below `max_accel_to_decel`, Klipper will not generate the traditional
   trapezoid, but will put deceleration right after acceleration. This is not
   much of a problem with acceleration_order > 2 because the transition between
   acceleration and deceleration is smooth anyways (toolhead acceleration is 0
   at the switching point).
 * Instant toolhead acceleration may exceed the configured `max_accel` value.
   `max_accel` serves as a limit for the 'average' acceleration during the move
   with acceleration_order > 2. Still, there is typically no need to reduce
   it, unless the steppers start skipping steps.
 * S-Curve acceleration mode puts less stress on the extruder when pressure
   advance is enabled (unless `max_jerk` is set to a too high value).
 * Klipper stops the toolhead between extrude and non-extrude moves, which, in
   case of S-Curve acceleration, can additionally slow down the prints. Zig-zag
   top/bottom pattern (line/rectilinear in Slic3r), as well as
   'Connect infill lines' setting in Cura can be used when slicing models
   to mitigate this issue to some degree. When these options are enabled,
   slicers will generate mostly connected lines at the top, bottom and infill,
   preventing toolhead from stopping for most of the moves.
