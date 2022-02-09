#!/bin/bash

if [ "$1" = "-h" ] || [ "$1" = "" ]
then
	printf "Usage:\t$0 [dataset path]\n"
	printf "\t$0 [dataset path] [arguments]\n"
	exit
fi

dataset_path=$1
arguments=$2
run_confirmation=""
cpu_cores=$(grep -c ^processor /proc/cpuinfo)
process_counter_max=$(($cpu_cores - 1))
process_counter=0
batch_counter=0

if [ "$dataset_path" = "" ] || [ ! -d "$dataset_path" ]
then
	echo "[ERROR]: Invalid dataset path \"$dataset_path\". Quit."
	exit
fi

read -e -p "[INFO]: Confirm running Depth Clustering on all datasets in the given path [y/N]: " run_confirmation

if [ "$run_confirmation" != "y" ] && [ "$run_confirmation" != "Y" ]
then
	echo "[INFO]: Quit."
    exit
fi

for dataset_path_segment in $(ls -d $dataset_path/segment-*/)
do
    ../../install/amd64/depth_clustering/release/bin/depth_clustering $(readlink -m "$dataset_path_segment") $arguments &
	pids[${i}]=$!

	echo "[INFO]: Launched batch $batch_counter process $process_counter."

	if [ "$process_counter" -ge "$process_counter_max" ]
	then
		for pid in ${pids[*]}
		do
			wait $pid
		done

		process_counter=0
		((batch_counter++))
	fi

	((process_counter++))
done
