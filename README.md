# LabelFeederArduino

Arduino Mega

TMC2209

L293D


[Arduino Hat Board](https://oshwlab.com/mateusz.przybyl.mp/powerdistributor2)

[DB25 Distributor](https://oshwlab.com/mateusz.przybyl.mp/powerdistributor)

## Error list

ERROR<00>: Unknown command

ERROR<01>: Limit switch was triggered

ERROR<02>: Stepper motor software limit

ERROR<03>: Low Vacuum during swing movement

ERROR<04>: Swing Blocked

ERROR<05>: Low vacuum on label pickup

## State list

STATE<00>: ERROR

STATE<01>: IDLE

## Commands

RESET

NEXT_LABEL

JSON


## Block diagram

```
           CMD> RESET          CMD> NEXT_LABEL
                  |                    |
*Power ON* ---> ERROR ---> RESET ---> IDLE --->  FIRST_LABEL  --->  NEXT_LABEL ---> FIRST_LABEL
                                       ┃              ┇                                  ┃
                                       ┃   (only after RESET state)                      ┃
                                       ┃              ┇                                  ┃
                                       ┗━━━━━<<<━━━━━━┻━━━━━━━━━━━━━━━━<<<━━━━━━━━━━━━━━━┙
```

## JSON Default Settings

| Variable            | Value |
| ------------------- | ----- |
| STEPS_PER_MM        | 160   |
| RUN_CURRENT_PERCENT | 100   |
| MAX_TRAVEL          | 120   |
| MAX_SPEED           | 8     |
| HOMING_SPEED        | 3     |
| MAX_ACCELERATION    | 2500  |
| VACUUM_DELAY        | 1300  |
| PROBE_DELAY         | 500   |
| CLAMP_DELAY         | 300   |
| VACUUM_THRESHOLD    | 250   |
