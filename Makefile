.PHONY: bash, build, run_simulation

DEV_DOCKERFILE = ".docker/dev/Dockerfile"
DOCKER_RUN_OPTIONS = \
	--name=navigation-python \
	--rm \
	--env="DISPLAY=${DISPLAY}" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="${XAUTHORITY}:/root/.Xauthority" \
	--volume="${PWD}:/src" \
	navigation-python

build:
	docker build -t navigation-python -f ${DEV_DOCKERFILE} .

run_simulation:
	docker run -it ${DOCKER_RUN_OPTIONS} bash -c "python -m navigation.run_simulation"

run_gui:
	docker run -it ${DOCKER_RUN_OPTIONS} bash -c "python -m navigation.visualization.gui"

bash:
	docker run -it ${DOCKER_RUN_OPTIONS} bash
