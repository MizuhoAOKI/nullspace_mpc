# Usage: make [command]
SHELL:=/bin/bash
WORKSPACE=$(shell pwd)

.PHONY: build # to avoid error

build:
	@set -e; \
	source /opt/ros/noetic/setup.bash; \
	export CC=clang-11 CXX=clang++-11; \
	ARCH=$$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || echo x86_64-linux-gnu); \
	catkin build --cmake-args \
	  -DCMAKE_BUILD_TYPE=Release \
	  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
	  -DCMAKE_SYSTEM_LIBRARY_PATH="/usr/lib/$$ARCH;/lib/$$ARCH;/usr/lib;/lib" \
	  -DCMAKE_CXX_FLAGS="-O2" \
	  -DENABLE_OSQP=ON

clean:
	rm -r build devel logs .catkin_tools

# install_deps: # install packages which are not supported by rosdep
install_deps:
	apt update && apt install -y \
		git \
		cmake \
		build-essential \
		pkg-config \
		psmisc \
		clang-11
	bash shell/ensure_cmake.sh 3.18.0
	bash shell/install_osqp.sh
	bash shell/install_osqp_eigen.sh

setup_docker:
	docker build -t noetic_image:latest -f docker/Dockerfile . --no-cache

exec_docker:
	docker exec -it noetic_container /ros_entrypoint.sh /bin/bash

run_rocker:
	rocker --x11 --user --network host --privileged --nocleanup --volume .:/home/$(shell whoami)/nullspace_mpc --name noetic_container noetic_image:latest

run_docker:
	@if [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "running" ]; then \
		$(MAKE) exec_docker; \
	elif [ "$(shell docker inspect --format='{{.State.Status}}' noetic_container)" = "exited" ]; then \
		docker start noetic_container; \
		if [$? -eq 0]; then \
			$(MAKE) exec_docker; \
		else \
			docker rm noetic_container; \
			$(MAKE) run_rocker; \
		fi; \
	else \
		$(MAKE) run_rocker; \
	fi

killall:
	./shell/killall.sh

# record rosbag (all topics)
record:
	cd ${WORKSPACE}/rosbag; rosbag record -a

# play and check rosbag
## [shell 1] make view_rosbag
## [shell 2] rosbag play rosbag/xxx.bag
view_rosbag:
	source /opt/ros/noetic/setup.bash && source ./devel/setup.bash &&\
	roslaunch launch/rosbag_play.launch workspace:=${WORKSPACE}

# gazebo_world.launch
gazebo_world:
	source /opt/ros/noetic/setup.bash && source ./devel/setup.bash &&\
	roslaunch launch/gazebo_world.launch

# gmapping.launch
gmapping:
	source /opt/ros/noetic/setup.bash && source ./devel/setup.bash &&\
	roslaunch launch/gmapping.launch workspace:=${WORKSPACE}

# navigation.launch
navigation:
	source /opt/ros/noetic/setup.bash && source ./devel/setup.bash &&\
	roslaunch launch/navigation.launch workspace:=${WORKSPACE}

# navigation with nullspace_mpc
navigation_nullspace_mpc:
	source /opt/ros/noetic/setup.bash && source ./devel/setup.bash &&\
	roslaunch launch/navigation.launch workspace:=${WORKSPACE} local_planner:=nullspace_mpc

# navigation with mppi
navigation_mppi:
	source /opt/ros/noetic/setup.bash && source ./devel/setup.bash &&\
	roslaunch launch/navigation.launch workspace:=${WORKSPACE} local_planner:=mppi_h

# evaluation demo with nullspace_mpc
eval_demo_nullspace_mpc:
	source $(WORKSPACE)/devel/setup.bash &&\
	mkdir -p result &&\
	python3 $(WORKSPACE)/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py \
	--agenda_yaml_path $(WORKSPACE)/data/eval_demo/agenda.yaml \
	--controller nullspace_mpc

# evaluation demo with mppi
eval_demo_mppi:
	source $(WORKSPACE)/devel/setup.bash &&\
	mkdir -p result &&\
	python3 $(WORKSPACE)/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py \
	--agenda_yaml_path $(WORKSPACE)/data/eval_demo/agenda.yaml \
	--controller mppi_h

# eval_demo_mppi_3d_a:
# 	source $(WORKSPACE)/devel/setup.bash &&\
# 	mkdir -p result &&\
# 	python3 $(WORKSPACE)/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py \
# 	--agenda_yaml_path $(WORKSPACE)/data/eval_demo/agenda.yaml \
# 	--controller mppi_3d_a

# eval_demo_mppi_3d_b:
# 	source $(WORKSPACE)/devel/setup.bash &&\
# 	mkdir -p result &&\
# 	python3 $(WORKSPACE)/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py \
# 	--agenda_yaml_path $(WORKSPACE)/data/eval_demo/agenda.yaml \
# 	--controller mppi_3d_b

eval_ten_small:
	source $(WORKSPACE)/devel/setup.bash &&\
	mkdir -p result &&\
	python3 $(WORKSPACE)/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py \
	--agenda_yaml_path $(WORKSPACE)/data/eval_ten/small/agenda_small.yaml \

eval_ten_large:
	source $(WORKSPACE)/devel/setup.bash &&\
	mkdir -p result &&\
	python3 $(WORKSPACE)/src/evaluation/mpc_nav_evaluator/scripts/run_evaluation.py \
	--agenda_yaml_path $(WORKSPACE)/data/eval_ten/large/agenda_large.yaml \
