# Defining Fairness in Human-Robot Teams

This respository contains code for the following paper:

[Chang, M. L., Pope, Z., Short, E. S., & Thomaz, A. L. (2020, August). Defining fairness in human-robot teams. In 2020 29th IEEE International Conference on Robot and Human Interactive Communication (RO-MAN) (pp. 1251-1258). IEEE.](https://ieeexplore.ieee.org/abstract/document/9223594)


## Instructions:

### Start robot arm:
```
python mdp_arm_controller.py
```

### Run practice session:
```
python cleaning_sim_GUI_r2.py EASY 4
```

### Run study session:
Each study session is characterized by two factors: math level and study condition.

The math levels are: EASY, MEDIUM, HARD. Each participant's math level is determined from the practice session.

The study conditions are:

* 0: Fluency-Absent & Effort-Absent
* 1: Fluency-Present & Effort-Absent
* 2: Fluency-Absent & Effort-Present
* 3: Fluency-Present & Effort-Present

For example, the study condition with Fluency-Present & Effort-Present and math level medium is:
```
python cleaning_sim_GUI_r2.py MEDIUM 3
```

## Bibliography
If you find this repository is useful in your research, please cite the paper:
```
@inproceedings{chang2020defining,
  title={Defining fairness in human-robot teams},
  author={Chang, Mai Lee and Pope, Zachary and Short, Elaine Schaertl and Thomaz, Andrea Lockerd},
  booktitle={2020 29th IEEE International Conference on Robot and Human Interactive Communication (RO-MAN)},
  pages={1251--1258},
  year={2020},
  organization={IEEE}
}
```
