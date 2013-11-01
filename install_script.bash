#!/bin/bash

if env | grep -q ^SCR_DIR
then
	echo "Already run setup"
else


	echo "installing realpath..."
	sudo apt-get install -y realpath

	echo "modifying bashrc file"
	export SCR_DIR=$PWD/../../

	echo "export SCR_DIR=$(realpath $SCR_DIR)" >> ~/.bashrc

	echo "Setup env variables successfully... source ~/.bashrc"
	source ~/.bashrc
fi
