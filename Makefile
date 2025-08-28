# Usage: make [command]
# --- Variables ---
SHELL:=/bin/bash
PROJ_NAME=nullspace_mpc
VERSION=0.1.1
USER_NAME=noetic

# Docker image and container naming
DOCKER_IMAGE_BASE = $(PROJ_NAME):$(VERSION)
CONTAINER_NAME_BASE = $(PROJ_NAME)-container

# Workspace and X11 forwarding settings
WORKSPACE = $(shell pwd)
XSOCK = /tmp/.X11-unix
XAUTH = /tmp/.docker.xauth

# Phony targets to prevent conflicts with file names
.PHONY: build

# Build CPU image
setup_docker_cpu:
	docker build \
		--build-arg BASE_IMAGE=ubuntu:20.04 \
		--build-arg ROS_PACKAGE=ros-noetic-desktop \
		-t $(DOCKER_IMAGE_BASE)-cpu \
		-f docker/Dockerfile_cpu .

# Build GPU image
setup_docker_gpu:
	docker build \
		-t $(DOCKER_IMAGE_BASE)-gpu \
		-f docker/Dockerfile_gpu .

# Launch or attach to the CPU container
run_docker_cpu:
	@CONTAINER="$(CONTAINER_NAME_BASE)-cpu"; \
	if [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "running" ]; then \
		echo "Attaching to running container: $$CONTAINER"; \
		$(MAKE) exec_docker_cpu; \
	elif [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "exited" ]; then \
		echo "Restarting and attaching to container: $$CONTAINER"; \
		docker start $$CONTAINER && $(MAKE) exec_docker_cpu; \
	elif [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "created" ]; then \
		echo "Starting and attaching to container: $$CONTAINER"; \
		docker start $$CONTAINER && $(MAKE) exec_docker_cpu; \
	elif [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "paused" ]; then \
		echo "Unpausing and attaching to container: $$CONTAINER"; \
		docker unpause $$CONTAINER && $(MAKE) exec_docker_cpu; \
	else \
		echo "Creating and running new container: $$CONTAINER"; \
		docker run -it --name $$CONTAINER \
			--network host \
			--privileged \
			--ipc host \
			--volume=$(WORKSPACE):/home/$(USER_NAME)/$(PROJ_NAME):rw \
			--volume=$(XSOCK):$(XSOCK):rw \
			--env="DISPLAY=$(DISPLAY)" \
			--env="QT_X11_NO_MITSHM=1" \
			$(DOCKER_IMAGE_BASE)-cpu \
			bash; \
	fi

# Launch or attach to the GPU container
run_docker_gpu:
	@CONTAINER="$(CONTAINER_NAME_BASE)-gpu"; \
	if [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "running" ]; then \
		echo "Attaching to running container: $$CONTAINER"; \
		$(MAKE) exec_docker_gpu; \
	elif [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "exited" ]; then \
		echo "Restarting and attaching to container: $$CONTAINER"; \
		docker start $$CONTAINER && $(MAKE) exec_docker_gpu; \
	elif [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "created" ]; then \
		echo "Starting and attaching to container: $$CONTAINER"; \
		docker start $$CONTAINER && $(MAKE) exec_docker_gpu; \
	elif [ "$$(docker inspect -f '{{.State.Status}}' $$CONTAINER 2>/dev/null)" = "paused" ]; then \
		echo "Unpausing and attaching to container: $$CONTAINER"; \
		docker unpause $$CONTAINER && $(MAKE) exec_docker_gpu; \
	else \
		echo "Creating and running new container: $$CONTAINER"; \
		[ -e "$(XAUTH)" ] || install -m 600 /dev/null "$(XAUTH)"; \
		chmod 644 "$(XAUTH)" || true; \
		xauth nlist "$(DISPLAY)" | sed -e 's/^..../ffff/' | xauth -f "$(XAUTH)" nmerge - || true; \
		sudo chmod 777 $(XAUTH) && \
		docker run -it --name $$CONTAINER \
			--cap-add=SYS_NICE \
			--gpus all \
			--network host \
			--privileged \
			--ipc host \
			--shm-size=1gb \
			--volume=$(WORKSPACE):/home/$(USER_NAME)/$(PROJ_NAME):rw \
			--volume=$(XSOCK):$(XSOCK):rw \
			--volume=$(XAUTH):$(XAUTH):rw \
			--env=TERM=xterm-256color \
			--env="DISPLAY=$(DISPLAY)" \
			--env="XAUTHORITY=$(XAUTH)" \
			--env="QT_X11_NO_MITSHM=1" \
			--env="NVIDIA_VISIBLE_DEVICES=all" \
			--env="NVIDIA_DRIVER_CAPABILITIES=all" \
			--env="MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA" \
			$(DOCKER_IMAGE_BASE)-gpu \
			bash; \
	fi

# Attach to the running CPU container
exec_docker_cpu:
	docker exec -it $(CONTAINER_NAME_BASE)-cpu bash

# Attach to the running GPU container
exec_docker_gpu:
	docker exec -it $(CONTAINER_NAME_BASE)-gpu bash

# build ros packages
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

# clean build caches
clean:
	rm -r build devel logs .catkin_tools

# install packages which are not supported by rosdep
install_deps:
	sudo apt-get update && apt-get install -y \
		git \
		cmake \
		build-essential \
		pkg-config \
		psmisc \
		clang-11
	bash shell/ensure_cmake.sh 3.18.0
	bash shell/install_osqp.sh
	bash shell/install_osqp_eigen.sh

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
