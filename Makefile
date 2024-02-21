.PHONY: release debug install upload

# name (from conan package)
NAME := $(shell python -c 'from conanfile import Project; print(Project.name)')

# version (from git branch or tag)
left := (
right := )
BRANCH := $(shell git diff-index --quiet HEAD && git tag --points-at HEAD)
ifeq ($(BRANCH),)
	BRANCH := $(shell git rev-parse --abbrev-ref HEAD)
endif
VERSION := $(subst /,-,$(subst $(left),_,$(subst $(right),_,$(BRANCH))))

# name/version@user/channel
#REFERENCE := $(NAME)/$(VERSION)@
REFERENCE := --version $(VERSION)

# options
export CONAN_RUN_TESTS=1
export CONAN_INSTALL_PREFIX=${HOME}/.local
RELEASE := -pr:b default -pr:h default -b missing -b $(NAME)
DEBUG := -pr:b debug -pr:h debug -b missing -b $(NAME)
ARMV6 := -pr:b default -pr:h armv6 -b missing -b $(NAME)
ARMV7 := -pr:b default -pr:h armv7 -b missing -b $(NAME)


# default target
all: remove native emu nrf52 stm32f0 stm32g4

remove:
	- conan remove -c $(NAME)/$(VERSION)

#debug:
#	conan create $(DEBUG) . $(REFERENCE)

#default:
#	conan create $(RELEASE) . $(REFERENCE)

native:
	conan create $(DEBUG) -o:a platform=native . $(REFERENCE)
#	conan create $(RELEASE) -o:a platform=native . $(REFERENCE)

emu:
	conan create $(DEBUG) -o:a platform=emu . $(REFERENCE)
#	conan create $(RELEASE) -o:a platform=emu . $(REFERENCE)

nrf52:
	conan create $(ARMV7) -o:a platform=nrf52840 . $(REFERENCE)

stm32f0:
#	conan create $(ARMV6) -o:a platform=stm32f042x6 . $(REFERENCE)
	conan create $(ARMV6) -o:a platform=stm32f051x8 . $(REFERENCE)

stm32g4:
	conan create $(ARMV7) -o:a platform=stm32g431xx . $(REFERENCE)
#	conan create $(ARMV7) -o:a platform=stm32g474xx . $(REFERENCE)


# install (e.g. to ~/.local)
install:
	conan install $(RELEASE) $(REFERENCE)

# upload package to conan repository
upload:
	conan upload $(REFERENCE) --all --force
