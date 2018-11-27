#!/bin/bash
#TUM ICS
#Emmanuel Dean dean@tum.de
#Florian Bergner florian.bergner@tum.de

###############################################################################
#   Convert all .bag files in 'bags' folder to .csv files in 'csv' folder
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

# Two arguments:
# The bag pathFile, and the list of topics
if [ ! $# -eq 2 ]; then
    echoc BRED "ERROR: You must specify first the bagfile path/name and then the topic list. (txt file):"
    echoc BWHITE "rosbag2csv path/file.bag listofTopics.txt"
    exit 1
fi

bagFile=$1
topicsFile=$2

if [ ! -f $bagFile ]; then
    echoc BRED "ERROR: bagfile ${bagFile} not found!"
    exit 1
fi

if [ ! -f $topicsFile ]; then
    echoc BRED "ERROR: topics file not found: ${topicsFile}."    
    exit 1
fi

echoc YELLOW "BagFile: " BYELLOW "${bagFile}"
echoc GREEN "TopicsFile: " BGREEN "${topicsFile}"

#Read the list of topics 
topics=($(cat $topicsFile))
#Get number of topics
topicsLen=${#topics[@]}

pids=()

bagFileName=$(basename "$bagFile")
bagFileName="${bagFileName%.*}"

echoc BGREEN "Get topics from bag: '$bagFileName'"

for (( j=0; j<topicsLen; j++ ))
do
    #Extract the topics one by one
    topic=${topics[${j}]}
    #Replace / with _ in the topic name
    topicExt=$(echo "$topic" | tr / _)

    csvFileName="$bagFileName$topicExt.csv"

    #Create containing folder for the csv file and the reMap template file
    mkdir $currentDir/$bagFileName$topicExt

    (rostopic echo -b $bagFile -p $topic > $currentDir/$bagFileName$topicExt/$csvFileName) &

    #Get the pid of the prevoiusly called command (rostopic in this case)
    pid=$! 
    pids+=($pid)

    echo " -- topic: $topic ------> $csvFileName (pid: $pid)"

    #Create a template file with the fields remapping
    echo "#name_of_field_in_csv_file:variable_name" >  "$currentDir/$bagFileName$topicExt/templateReMap.txt"
    echo "#e.g." >>  "$currentDir/$bagFileName$topicExt/templateReMap.txt"
    echo "field.header.stamp:t" >>  "$currentDir/$bagFileName$topicExt/templateReMap.txt"
    echo "field.q1:q1" >>  "$currentDir/$bagFileName$topicExt/templateReMap.txt"

    
    # Create a template with plot descriptions (how data extacted from rosbags will be plotted)
    str=">Plot *CpuTemp* {title1}(signal1 [units1]){tilte2}(signal2 [units2],signalL [unitsL]) 1\n" 
    str+="ctrlCpuInfoDS.cpuTotal,ctrlCpuInfoDS.t:mediaCpuInfoDS.cpuTemp,mediaCpuInfoDS.t;ctrlCpuInfoDS.cpuTemp\n"
    str+=">Plot *CpuTotal* {title1}(signal1 [units1]){tilte2}(signal2 [units2]) 2\n"
    str+="mediaCpuInfoDS.cpuTotal,mediaCpuInfoDS.t\n"
    str+="mediaCpuInfoDS.cpuTemp,mediaCpuInfoDS.t\n"
    str+=">Plot *CpuTemp1X1* {title1}(signal1 [units1]) 3\n"
    str+="mediaCpuInfoDS.cpuTotal,mediaCpuInfoDS.t\n"
    str+=">Plot *CpuTemp2X2* {title1}(signal1 [units1]){tilte2}(signal2 [units2]){title3}(signal3 [units3]){tilte4}(signal4 [units4]) 4\n"
    str+="mediaCpuInfoDS.cpuTotal,mediaCpuInfoDS.t:ctrlCpuInfoDS.cpuTemp,ctrlCpuInfoDS.t\n"
    str+="ctrlCpuInfoDS.cpuTemp,ctrlCpuInfoDS.t:ctrlCpuInfoDS.cpuTotal,ctrlCpuInfoDS.t\n"
    str+=">Plot *CpuTotalQ* {title1}(signal1 [units1]){tilte2}(signal2 [units2]){title3}(signal3 [units3]){tilte4}(signal4 [units4]) 5\n"
    str+="mediaCpuInfoDS.cpuTotal,mediaCpuInfoDS.t:ctrlCpuInfoDS.cpuTemp,ctrlCpuInfoDS.t\n"
    str+="ctrlCpuInfoDS.cpuTemp,ctrlCpuInfoDS.t:h1DataDS.q1,h1DataDS.t,h1DataDS.q2,h1DataDS.t,h1DataDS.q3,h1DataDS.t,h1DataDS.q4,h1DataDS.t\n"
    str+=">"
    echo -e $str > "$currentDir/$bagFileName$topicExt/templatePlot.txt"
    
done


cleanup()
{
    len=${#pids[@]}
    for (( i=0; i<len; i++ ))
    do
        pid=${pids[${i}]}

        echo "kill $pid"
        kill $pid
    done
    exit 0
}

# catch Ctrl+C to terminate properly all the running threads (processes)
trap "cleanup" 2
#trap "cleanup" EXIT


len=${#pids[@]}
lenFinished=0

while [ $lenFinished -ne $len ]
do
    lenFinished=0
    
    echo "---- status ----"
    for (( i=0; i<len; i++ ))
    do
        pid=${pids[${i}]}
        
        #Check if the process is still running 
        if ps -p $pid > /dev/null
        then
            echo "$pid is running"
        else
            #if the process has finished increase the len of finished processes --lenFinished--
            echo "$pid finished"
            ((lenFinished++))
        fi
    done
    sleep 1.0
    
    # delete previous output
    # clear one line
    tput cuu1
    
    for (( i=0; i<len; i++ ))
    do
        tput cuu1
    done
    #clear to end of screen
    tput ed
done
