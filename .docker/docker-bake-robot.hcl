#
# Override these variables with environment variables
# e.g.
#
#   BLUE_ROS_DISTRO=iron docker buildx bake
#
# or
#
#   export BLUE_ROS_DISTRO=iron
#   docker buildx bake
#
variable "BLUE_ROS_DISTRO" { default = "jazzy" }
variable "BLUE_GITHUB_REPO" { default = "patpat98/bubble_blue" }

group "default" {
  targets = ["ci", "robot"]
}

# These are populated by the metadata-action Github action for each target
# when building in CI
#
target "docker-metadata-action-ci" {}
target "docker-metadata-action-robot" {}
target "docker-metadata-action-desktop" {}
target "docker-metadata-action-desktop-nvidia" {}


#
# All images can pull cache from the images published at Github
# or local storage (within the Buildkit image)
#
# ... and push cache to local storage
#
target "ci" {
  inherits = ["docker-metadata-action-ci"]
  dockerfile = ".docker/Dockerfile_robot"
  target = "ci"
  context = "."
  args = {
    ROS_DISTRO = "${BLUE_ROS_DISTRO}"
  }
  tags = [
    "ghcr.io/${BLUE_GITHUB_REPO}:arm64-${BLUE_ROS_DISTRO}-ci"
  ]
  labels = {
    "org.opencontainers.image.source" = "https://github.com/${BLUE_GITHUB_REPO}"
  }
  cache_from =[
    "ghcr.io/${BLUE_GITHUB_REPO}:cache-${BLUE_ROS_DISTRO}-ci",
    "ghcr.io/${BLUE_GITHUB_REPO}:cache-${BLUE_ROS_DISTRO}-robot",
    "ghcr.io/${BLUE_GITHUB_REPO}:cache-${BLUE_ROS_DISTRO}-desktop",
    "ghcr.io/${BLUE_GITHUB_REPO}:cache-${BLUE_ROS_DISTRO}-desktop-nvidia",
    "type=local,dest=.docker-cache"
  ]
  cache_to = [
    "type=local,dest=.docker-cache"
  ]
  platforms = ["linux/arm64"]
}

target "robot" {
  inherits = [ "ci", "docker-metadata-action-robot" ]
  target = "robot"
  tags = [
    "ghcr.io/${BLUE_GITHUB_REPO}:arm64-${BLUE_ROS_DISTRO}-robot"
  ]
  cache_to = [
    "type=local,dest=.docker-cache"
  ]
}
