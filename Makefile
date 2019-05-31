build:
	bash -c "docker build . -t pascalau/apriltags3:latest"
push:
	bash -c "docker push pascalau/apriltags3:latest"

run-%:
	bash -c "docker -H $*.local rm apriltags3" || echo "no apriltags 3 found"
	bash -c "docker -H $*.local pull pascalau/apriltags3:latest"
	bash -c "docker -H $*.local run -it --net host --privileged -v /data:/data --name apriltags3 pascalau/apriltags3:latest"
