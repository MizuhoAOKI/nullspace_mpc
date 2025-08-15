#!/bin/bash

# settings
EPISODE_NUM=hundred # ten or hundred
WORKSPACE=~/nullspace_mpc
AGENDA_SMALL=$WORKSPACE/data/eval_$EPISODE_NUM/small/agenda_small.yaml
AGENDA_LARGE=$WORKSPACE/data/eval_$EPISODE_NUM/large/agenda_large.yaml

# source the project
source $WORKSPACE/devel/setup.bash;

# large in cylinder_garden
## run nullspace_mpc
source $WORKSPACE/devel/setup.bash &&\
mkdir -p result &&\
python3 $WORKSPACE/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py --agenda_yaml_path $AGENDA_LARGE --controller nullspace_mpc --world_name cylinder_garden;

## run mppi_h
source $WORKSPACE/devel/setup.bash &&\
mkdir -p result &&\
python3 $WORKSPACE/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py --agenda_yaml_path $AGENDA_LARGE --controller mppi_h --world_name cylinder_garden;

# large in maze
## run nullspace_mpc
source $WORKSPACE/devel/setup.bash &&\
mkdir -p result &&\
python3 $WORKSPACE/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py --agenda_yaml_path $AGENDA_LARGE --controller nullspace_mpc --world_name maze;

## run mppi_h
source $WORKSPACE/devel/setup.bash &&\
mkdir -p result &&\
python3 $WORKSPACE/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py --agenda_yaml_path $AGENDA_LARGE --controller mppi_h --world_name maze;

# announce
echo "#####";
echo "All evaluations are finished.";
echo "#####";
