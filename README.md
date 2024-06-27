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

| Variable         | Value                  |
| ---------------- | ---------------------- |
| STEPS_PER_MM     | 160                    |
| MAX_TRAVEL       | (STEPS_PER_MM \* 120)  |
| MAX_SPEED        | (STEPS_PER_MM \* 15)   |
| HOMING_SPEED     | (STEPS_PER_MM \* 3)    |
| MAX_ACCELERATION | (STEPS_PER_MM \* 2500) |
| VACUUM_DELAY     | 400                    |
| PROBE_DELAY      | 500                    |
| CLAMP_DELAY      | 300                    |
| VACUUM_THRESHOLD | 300                    |
