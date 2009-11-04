# Welcome to the Makefile for the NUbot Code
#
#    Copyright (c) 2009 Jason Kulk
#    This file is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This file is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
# Targets: NAO, NAOWebots, Cycloid

CUR_DIR = $(shell pwd)

# Make directories
MAKE_DIR = $(CUR_DIR)/Make
MAKE_OPTIONS = --no-print-directory

# Build directories
NAO_BUILD_DIR = Build/NAO
NAOWEBOTS_BUILD_DIR = Build/NAOWebots
CYCLOID_BUILD_DIR = Build/Cycloid

# Aldebaran build tools
ALD_CC_SCRIPT = $(AL_DIR)/tools/crosscompile.sh
ALD_CTC = $(AL_DIR)/crosstoolchain

.PHONY: default_target all 
.PHONY: NAO NAOConfig NAOClean NAOVeryClean
.PHONY: NAOWebots NAOWebotsConfig NAOWebotsClean NAOWebotsVeryClean
.PHONY: Cycloid CycloidConfig CycloidClean CycloidVeryClean
.PHONY: clean veryclean

# We export an environment variable TARGET_ROBOT which tells everything
# which robot we want this build to run on.
TARGET_ROBOT = NAOWEBOTS				# by default the target is Webots
NAOWebots: TARGET_ROBOT=NAOWEBOTS
NAOWebotsConfig: TARGET_ROBOT=NAOWEBOTS
NAO: TARGET_ROBOT=NAO
NAOConfig: TARGET_ROBOT=NAO
Cycloid: TARGET_ROBOT=CYCLOID
CycloidConfig: TARGET_ROBOT=CYCLOID
export TARGET_ROBOT

default_target: NAOWebots

all: NAO NAOWebots Cycloid

NAO:
	@tput setf 1;
	@echo "Targetting NAO";
	@tput setf 7
	@if [ -f $(CUR_DIR)/$(NAO_BUILD_DIR)/Makefile ]; then \
		echo "Already configured"; \
		cd $(NAO_BUILD_DIR); \
		tput sgr0; \
		make $(MAKE_OPTIONS); \
	else \
		mkdir -p $(NAO_BUILD_DIR); \
		cd $(NAO_BUILD_DIR); \
		sh $(ALD_CC_SCRIPT) $(ALD_CTC) $(MAKE_DIR); \
		ccmake .; \
		tput setf 1; \
		echo "Configuration complete"; \
		tput sgr0; \
		make $(MAKE_OPTIONS); \
	fi

NAOConfig:
	@echo "Configuring NAO Build";
	@set -e; \
		cd $(NAO_BUILD_DIR); \
		sh $(ALD_CC_SCRIPT) $(ALD_CTC) $(MAKE_DIR); \
		ccmake .; \
		make $(MAKE_OPTIONS);

NAOClean:
	@echo "Cleaning NAO Build";
	@set -e; \
		cd $(NAO_BUILD_DIR); \
		make $(MAKE_OPTIONS) clean;

NAOVeryClean:
	@echo "Hosing NAO Build";
	@set -e; \
		cd $(NAO_BUILD_DIR); \
		rm -rf ./*;

NAOWebots:
	@echo "Targetting NAOWebots";
	@if [ -f $(CUR_DIR)/$(NAOWEBOTS_BUILD_DIR)/Makefile ]; then \
		cd $(NAOWEBOTS_BUILD_DIR); \
		make $(MAKE_OPTIONS); \
	else \
		mkdir -p $(NAOWEBOTS_BUILD_DIR); \
		cd $(NAOWEBOTS_BUILD_DIR); \
		cmake $(MAKE_DIR); \
		ccmake .; \
		make $(MAKE_OPTIONS); \
	fi

NAOWebotsConfig:
	@set -e; \
		cd $(NAOWEBOTS_BUILD_DIR); \
		cmake $(MAKE_DIR); \
		ccmake .; \
		make $(MAKE_OPTIONS);

NAOWebotsClean:
	@echo "Cleaning NAOWebots Build";
	@set -e; \
		cd $(NAOWEBOTS_BUILD_DIR); \
		make $(MAKE_OPTIONS) clean;

NAOWebotsVeryClean:
	@echo "Hosing NAOWebots Build";
	@set -e; \
		cd $(NAOWEBOTS_BUILD_DIR); \
		rm -rf ./*;

Cycloid:
	@echo "Targetting Cycloid";
	@cmake $(MAKE_DIR);

CycloidClean:
	@echo "Cleaning Cycloid Build"

CycloidVeryClean:
	@echo "Hosing Cycloid Build";
	@set -e;

clean: NAOClean NAOWebotsClean CycloidClean

veryclean: NAOVeryClean NAOWebotsVeryClean CycloidVeryClean


# Helpful tips:
# 1. Makefile uses actual tabs as separators, so you'll get a missing separator
#    with editors that use spaces instead
# 2. Make sure there is no white space after the \
