.PHONY: build, run

DEV_DOCKERFILE := .docker/dev/Dockerfile

build:
	docker build -t navigation-python -f ${DEV_DOCKERFILE} .

run:
	docker run -it \
		--name=navigation-python \
		--rm \
		--env="DISPLAY=${DISPLAY}" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="${XAUTHORITY}:/root/.Xauthority" \
		--volume="${PWD}:/src" \
		navigation-python bash -c "python src/models/vehicle.py"
