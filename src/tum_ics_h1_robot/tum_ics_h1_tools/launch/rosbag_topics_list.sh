#!/bin/bash

###############################################################################
#  To be executed in bags folder...
#
# Get topics to monitor from "topic.txt" file in folder parent folder and
# start to record topics into rosbag file.
#
###############################################################################

# get location of this script
sourceDir=$BASH_SOURCE
runDir=$0
thisDir=$(cd $(dirname $sourceDir); pwd)
currentDir=$(pwd)

if [[ $sourceDir != $runDir ]]; then
    echoc BRED "ERROR: You must run this script."
    exit 1
fi


if [ ! $# -eq 2 ]; then
    echoc BRED "ERROR: You must specify first the name and then the topic list."
fi

name=$1
topicFile=$2

if [ -z $name ]; then
    echoc BRED "ERROR: Empty file name."
fi

if [ ! -f $topicFile ]; then
    echoc BRED "ERROR: Invalid file for topic list '$topicFile'."    
fi

#echo "$0"
#echo "$1"

fileName="$name"

echo "Filename: $fileName"
echo "Topics from: $topicFile"

topics=($(cat $topicFile))
len=${#topics[@]}

unset topicList
for (( i=0; i<len; i++ ))
do
	topic=${topics[${i}]}
	topicList+="${topic} "
done

#echo $topicList

rosbag record -o $fileName $topicList



