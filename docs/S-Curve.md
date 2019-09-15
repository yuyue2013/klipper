S-Curve acceleration
====================

This document provides an overview of a special mode of acceleration implemented
in Klipper called S-Curve acceleration. This acceleration scheme replaces the
traditional "[trapezoid generator](Kinematics.md#trapezoid-generator)" by modeling higher-order derivatives, like
jerk, snap and so forth, depending on the S-Curve order, using Bezier polynomials.

**Warning**: S-Curve support is experimental, and some people reported that
higher order acceleration schemes make the quality of the prints worse. You
should consider using high order acceleration only if you already have some
problems with the prints, otherwise it is not advised to switch over from
the default acceleration_order = 2. Examples of problems S-Curve
acceleration may help with:

* Ghosting and ringing in the prints
* Extruder skipping steps or rattling when using
  [Pressure Advance](Pressure_Advance.md):
    * bowden extruder with high pressure advance value
    * direct extruder with high gear ratio (e.g. remote direct drive extruder)

Note that ghosting usually has mechanical origins: insufficiently rigid printer
frame, non-tight or too springy belts, alignment issues of mechanical parts,
heavy moving mass, etc. Those should be checked and fixed first. If you still
want to try S-Curve acceleration, know that your prints will likely slow down,
though usually not a lot. At any rate, some tuning is required to improve the
quality of the prints.

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
acceleration *a*<sub>e</sub> if Pressure Advance is enabled. Extruder kinematics
looks as follows in this case:

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

### How to enable S-Curve acceleration mode

Just add `acceleration_order` and `max_jerk` paramters to your `printer.cfg`
configuration file:

```
[printer]
acceleration_order: ...
max_jerk: ...
```

Below we provide some insights how to tune these (and other) parameters to
get better quality of your prints.

S-Curve acceleration tuning
===========================

The quality of the prints depends on the several kinematic parameters:

* Maximum velocity
* Acceleration
* [Square corner velocity](Kinematics.md#look-ahead)
* Acceleration order
* Maximum jerk (if S-Curve is enabled)

To enable S-Cuve acceleration, set acceleration order greater than 2. S-Curve
acceleration tuning consists of choosing the acceleration order > 2 and tuning
the maximum jerk value. The rest of the parameters should be already known,
preferably from the printer manufacturer, but we will provide a few suggestions.

### Maximum velocity

Maximum velocity depends on the number of factors, including the quality of the
rods or rails used in the printer, steppers used and the power supply voltage,
and the volumetric throughput of the hotend.

Naturally rods or rails used in the printer motion system must be able to handle the
speed.

Steppers produce EMF ([Electromotive force](https://en.wikipedia.org/wiki/Electromotive_force)) as they rotate, so the maximum attainable toolhead speed
depends on their parameters, power supply voltage and the printer kinematics
type. There is an excellent 'EMF Calculator' available at
[RepRapFirmware.org](http://www.reprapfirmware.org/) which can help to estimate
the maximum velocity.

Finally, the hotend can melt only a limited amount of plastic at a time.
For example, E3d V6 hotend has the max volumetric flow of around 10-15 mm^3/sec.
Dividing the max volumetric flow by the extrusion width and the layer height
gives the maximum toolhead velocity to not exceed the melting speed of the
hotened.

### Maximum acceleration

Maximum acceleration depends on a few factors such as the steppers torque, their
current, the moving mass and the stiffness of the printer frame in general. In
addition, Pressure Advance may be a limiting factor if enabled. The steppers
should not skip steps, and the printer frame or toolhead should not wobble as
the steppers apply force to accelerate or decelerate the toolhead. One can run
the [ringing test](#tuning-process) gradually increasing the acceleration
to find out which value is not too much for the quality.

### Square corner velocity

Square corner velocity defines the velocity limit for the toolhead to traverse
the square corners. It is used to determine the maximum velocity between the
moves with other angles as well. Setting too high values will require the
toolhead to change its direction at high speed, which, depending on the moving
mass, can cause printer wobbling and vibrations, resulting in ringing.

### Maximum jerk

Unlike other parameters, there is no well-defined upper limit on the maximum
jerk value, except when using Pressure Advance. As a general rule, the higher
`max_jerk` value (*J*) is, the shorter are the moves that do full acceleration.
For a simple case of acceleration from 0 speed, the relation is the following:

*J* = (18 *a*<sup>3</sup> / *L*)<sup>1/2</sup>.

For example, min max jerk values required to do a full acceleration of
500 mm/sec^2 on a segment of *L* = 10 mm, 1 mm and 0.1 mm are
*J* = 15'000, 47'434 and 150'000 respectively (mm/sec^3). Full acceleration at
3000 mm/sec^2 require *J* = 220'454, 697'137 and 2'204'540 respectively
(mm/sec^3). The time required to cover the move of length L, assuming it is
limited by jerk and that the initial speed is 0 is

*T* = (18 *L* / *J*)<sup>1/2</sup>.

As an example, 1 mm move will take 0.042 sec with *J* = 10'000 mm/sec^3 and
0.013 sec with *J* = 100'000 mm/sec^3.

There is another consideration if one uses Pressure Advance: toolhead jerk
directly translates into extruder acceleration: *a<sub>e</sub>* = *r P J*,
where *P* is the pressure advance parameter,

*r* = (4 *w* *h*) / (&pi; *D*<sup>2</sup>).

is the extrusion ratio, *D* is the filament diameter, *w* is the extrusion
width, *h* is the layer height. So, with *P* = 0.5, *w* = 0.4 mm, *h* = 0.2 mm,
*D* = 1.75 mm, and *J* = 100'000 the extruder acceleration is 1663 mm/sec^2 due
to jerk.

With that in mind, the practical range of values to test for `max_jerk` value
is from 10'000 mm/sec^3 to around 100'000 - 300'000 mm/sec^3 depending on the
printer setup and hardware.

### Tuning process

Start the process by modifying the following `[printer]` section to have
the following parameters:
```
[printer]
acceleration_order: 4
square_corner_velocity: 100
max_jerk: 200000
```
The idea is to put values that should not be exceeded during the testing.
If you also want to tune the acceleration, change it to be something like

```max_accel: 10000```

Prepare the [test](https://www.thingiverse.com/thing:3847206) model in the slicer:

 * Suggested layer height is 0.25 mm.
 * Infill and top layers can be set to 0.
 * Use 1-2 perimeters, or even better the smooth vase mode.
 * Use sufficiently high speed, around 80-100 mm/sec, for *external* perimeters.

During the testing, the tuned parameter can be changed every 5 mm Z height,
either manually by issuing G-Code commands, or by adding it to the generated
gcode file (e.g. every 20 layers with 0.25 mm layer height).

Before tuning, adjust the printer config using G-Code commands:
`SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=1` and
`SET_PRESSURE_ADVANCE ADVANCE=0`.

1. **Tune acceleration (optional)**:
issue the following G-Code command to start: `SET_VELOCITY_LIMIT ACCEL=500`.
Print the test model increasing the acceleration every 5 mm by
500-1000 mm/sec^2 (`SET_VELOCITY_LIMIT ACCEL=1000` and so forth).
It's OK if there is some ghosting near the notches that are close to each other,
but do notice if it appears elsewhere, e.g. along the *long* sides. Choose the
max acceleration value below that, and set it using
`SET_VELOCITY_LIMIT ACCEL=...` command.

2. **Tune square corner velocity**:
issue `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=1 JERK=15000` command to start.
Print the test model increasing the square corner velocity every 5 mm by
2-5 mm/sec (e.g. `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=5` and so on).
Notice when the ringing becomes apparent, and use the square corner velocity
value just before that: `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=...`

3. **Tune maximum jerk**: start with `SET_VELOCITY_LIMIT JERK=10000`. Print
the test model increasing the maximum jerk every 5 mm by ~10000 mm/sec^3
(`SET_VELOCITY_LIMIT JERK=20000` and increasing). Choose the value that shows
the best results overall: `SET_VELOCITY_LIMIT JERK=...`

4. **Tune acceleration order (optional)**: acceleration_order = 4 should be
fine for many users. If you want, you can also repeat step 3 with
`SET_VELOCITY_LIMIT ACCEL_ORDER=6` and see if it shows better results with
higher `max_jerk` value.

5. **Test pressure advance (optional)**: you should have a tuned pressure
advance value already
([instructions](Pressure_Advance.md#tuning-pressure-advance)). Set it up
using G-Code command `SET_PRESSURE_ADVANCE ADVANCE=...` and repeat the step 3
with the chosen acceleration order. Notice the max jerk value before the
extruder starts to rattle or skip steps. It is suggested to use the minimum of
the 2 values obtained at steps 3 and 5 as the final `max_jerk` value.

6. **Store the tuned values** in your `printer.cfg` file.

S-Curve acceleration notes
==========
 * S-Curve reduces acceleration of short moves, effectively slowing them down
    * this can be disabled by setting very high `max_jerk` value in the config,
      e.g. 1000000, but this is not advised
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
   preventing toolhead from stopping for most of the moves. This has the most
   positive effect if `square_corner_velocity` is set reasonably high.
 * S-Curve acceleration does not work well with `[bed_meshing]` feature
   currently. As mentioned above, S-Curve acceleration slows down short moves.
   Incidentally, bed meshing segments long moves into short ones to account for
   the bed surface height changes. This can lead to inconsistent acceleration
   across the print area depending on segmentation density. If you must use bed
   meshing, you can set fade_start < fade_end options in `[bed_meshing]` section
   to small values (i.e. a few millimeters) to stop Z-height adjustments early
   and set `max_jerk` to a reasonably high value (around 50000 or more) to make
   sure the print is not too slow during these initial few millimeters.