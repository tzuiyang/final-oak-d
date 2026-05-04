# Slide 2 Notes: Runtime Diagram

**One-minute script**

This slide shows how our code connects to the upstream Pupper monorepo.

The gray boxes are upstream Pupper components. We keep their joystick, E-stop, controller manager, and neural walking controller. The upstream `neural_controller` is important because it takes `/cmd_vel` and turns it into leg motion.

The cyan boxes are our contribution. We wrote the OAK-D perception and following layer:

```text
object_follower_node.py
mission_controller_node.py
web_ui_node.py
detector.py
follower.py
```

The flow is: OAK-D detects people and estimates `(x, y, z)`. The web UI lets the operator select `person`. The mission controller validates that selected target and publishes `/oakd/target`. Then `object_follower_node.py` runs the follower and publishes `/cmd_vel`.

`object_follower_node.py` also works as the AUTO/MANUAL mux. In AUTO, it uses the OAK-D follower. In MANUAL, it forwards joystick teleop commands. Safety still comes from the upstream Pupper monorepo through the normal joystick and E-stop path.

**Key points**

- Upstream Pupper monorepo: locomotion, controller manager, E-stop, legs.
- Our repo: OAK-D detection, UI target selection, mission logic, follower control.
- UI publishes: `/oakd/select_target = "person"`.
- Mission controller publishes: `/oakd/target = "person"`.
- Follower publishes: `/cmd_vel`.
- Upstream `neural_controller` converts `/cmd_vel` into walking.
