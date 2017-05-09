This package provides provides collective information related to the safety of the Robotino platform. It currently collects safety messages from the bumper, the airskin and the wrist by the topics

```
/wrist/wrist_safety
/bumper
/arm_bumper
```

and forwards a `Safety.msg` of the form

```
bool emergency_state
bool bumper_stop
bool airskin_stop
bool wrist_stop
```

via the `squirrel_safety` topic.

