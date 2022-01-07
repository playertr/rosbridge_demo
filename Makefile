
DOCKER_IMAGE_BASE=amarburg/rosbridge_demo
DOCKER_IMAGE_ROS=${DOCKER_IMAGE_BASE}_ros
DOCKER_IMAGE_CLIENT=${DOCKER_IMAGE_BASE}_client

images: build-ros-image build-client-image

build-ros-image: docker/Dockerfile_ros
	docker build . -f docker/Dockerfile_ros -t ${DOCKER_IMAGE_ROS}

build-client-image: docker/Dockerfile_client
	docker build . -f docker/Dockerfile_client -t ${DOCKER_IMAGE_CLIENT}


run-ros-image:
	docker run -it --rm -p 8080:8080 -p 9090:9090 ${DOCKER_IMAGE_ROS} \
				roslaunch ros_pose_gen pose_server.launch

.phony: build-ros-image build-client-image run-ros-image