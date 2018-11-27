#!/bin/bash

# you need to install 'libcap2-bin'

workspaceDir="/home/pal/deployed_ws"

dir=${workspaceDir}/share

files=$(find $dir -type f -name "*.sh")

len=${#files[@]}
for (( i=0; i<len; i++ ))
do
    file=${files[${i}]}

    echo "Change permissions of '$file'"
    chmod +x $file
done


dir=${workspaceDir}/lib/tum_ics_perf_eval_monitor

ind=0
files[$((ind++))]="$dir/conn_monitor_tsu3"
files[$((ind++))]="$dir/topic_conn_detector"
files[$((ind++))]="$dir/conn_monitor_topic"
files[$((ind++))]="$dir/conn_monitor_pid"

len=${#files[@]}
for (( i=0; i<len; i++ ))
do
    file=${files[${i}]}

    echo "Change permissions of '$file'"

    setcap cap_net_raw,cap_net_admin=eip "$file"
    getcap "$file"
done

# copy libs which are not found for some reasons when changing the permissions
cp deployed_ws/lib/libtum_ics_params.so /usr/lib
cp deployed_ws/lib/libtum_ics_perf_eval_monitor.so /usr/lib
cp deployed_ws/lib/libtum_ics_perf_eval_bridge.so /usr/lib


# copy turbostat
cp /home/pal/linux-tools/turbostat-linux-3.18.24/turbostat /usr/bin

