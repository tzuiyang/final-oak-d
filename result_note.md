# Result Notes

**One-minute script**

This plot shows a recorded indoor person-following run. The y-axis is the estimated distance from the OAK-D camera to the selected person.

That distance comes from our 3-D state estimate:

```text
P_obj = (x, y, z)
d_hat = sqrt(x^2 + y^2 + z^2)
```

The thin line is the raw OAK-D estimate, and the thicker line is a 5-point median smooth for visualization. The yellow dashed line is our desired following distance:

```text
target_distance = 0.5 m
```

The main result is that the closest approach was:

```text
d_min = 0.47 m
```

So the stop error was:

```text
|0.47 - 0.50| = 0.03 m
```

That means the robot stopped within 3 cm of the target distance. The run was about 60 seconds, with about an 8 Hz detection rate. Even with noisy depth estimates, the simple reactive controller was accurate enough to follow the person and stop near 0.5 m.

**Key points**

- Recorded indoor run.
- UI-selected target: `person`.
- Desired distance: `0.5 m`.
- Closest approach: `0.47 m`.
- Stop error: `0.03 m`.
- Detection rate: about `8 Hz`.
- Result: visual state estimate was good enough for person following.
