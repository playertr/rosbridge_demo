
DOCKER_IMAGE_BASE=playertr/rosbridge_demo
DOCKER_IMAGE_ROS=${DOCKER_IMAGE_BASE}_ros
DOCKER_IMAGE_CLIENT=${DOCKER_IMAGE_BASE}_client

#
# Tasks for building the ROS server and cpp client dockers images
#
images: build-ros-image build-client-image

build-ros-image: docker/Dockerfile_ros
	docker build . -f docker/Dockerfile_ros -t ${DOCKER_IMAGE_ROS}

build-client-image: docker/Dockerfile_client
	docker build . -f docker/Dockerfile_client -t ${DOCKER_IMAGE_CLIENT}

push:
	docker push ${DOCKER_IMAGE_ROS} && docker push ${DOCKER_IMAGE_CLIENT}


#
# Tasks to run the ROS server, and cpp client images
#
run-ros-image:
	docker run -it --rm -p 8080:8080 -p 9090:9090 ${DOCKER_IMAGE_ROS} \
				roslaunch ros_pose_gen pose_server.launch

run-client-image:
	docker run -it --net=host ${DOCKER_IMAGE_CLIENT} build/client 2

run-client-image-roslibpy:
	docker run -it --net=host ${DOCKER_IMAGE_CLIENT} ./roslibpy_client.py


.phony: build-ros-image build-client-image run-ros-image push